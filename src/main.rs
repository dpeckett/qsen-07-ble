#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Timer};

use esp_hal::{
    clock::CpuClock,
    efuse::Efuse,
    i2c::master::{Config as I2cConfig, I2c},
    time::{Duration, Rate},
    timer::timg::TimerGroup,
};
use esp_wifi::{EspWifiController, ble::controller::BleConnector};

use core::cell::RefCell;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use heapless::{String, format};
use libm::roundf;
use log::{info, warn};
use trouble_host::prelude::ExternalController;
use trouble_host::prelude::*;
use {esp_alloc as _, esp_backtrace as _};

use crate::ags10::Ags10;
use aht20_driver::{AHT20, SENSOR_ADDRESS};
use bh1750::{BH1750, Resolution};
use bme280::i2c::BME280;
use scd4x::Scd4x;
use static_cell::StaticCell;

mod ags10;

/// Bluetooth advertising name
const ADVERTISED_NAME_PREFIX: &str = "Qsen-07";
/// Max number of connections
const CONNECTIONS_MAX: usize = 1;
/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att (as required by GATT)
/// Max number of command slots for the controller.
const COMMAND_SLOTS: usize = 20;

// ~20% of light makes it through the enclosure.
const LIGHT_TRANSMISSION_FACTOR: f32 = 0.2f32;

/// Bluetooth advertising name
static DEVICE_NAME: StaticCell<String<32>> = StaticCell::new();

esp_bootloader_esp_idf::esp_app_desc!();

type Mwdt0 = esp_hal::timer::timg::Wdt<esp_hal::peripherals::TIMG0<'static>>;
static WDT_CELL: StaticCell<Mwdt0> = StaticCell::new();

#[embassy_executor::task]
async fn watchdog_task(
    wdt: &'static mut esp_hal::timer::timg::Wdt<esp_hal::peripherals::TIMG0<'static>>,
) {
    loop {
        // Feed every 5 seconds. If tasks stall and this stops running,
        // MWDT will eventually expire and reset the chip.
        Timer::after_secs(5).await;
        wdt.feed();
    }
}

#[embassy_executor::task]
async fn ble_runner_task(
    runner: &'static mut Runner<
        'static,
        ExternalController<BleConnector<'static>, COMMAND_SLOTS>,
        DefaultPacketPool,
    >,
) {
    info!("[ble_runner_task] starting");
    loop {
        if let Err(e) = runner.run().await {
            panic!("[ble_runner_task] error: {:?}", e);
        }
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz));
    esp_alloc::heap_allocator!(size: 72 * 1024);

    // --- Wi-Fi/BLE + Embassy time base ---
    // Take both timer0 (for esp-wifi) and the watchdog from TIMG0.
    let TimerGroup {
        mut wdt, timer0, ..
    } = TimerGroup::new(peripherals.TIMG0);
    let init = esp_wifi::init(timer0, esp_hal::rng::Rng::new(peripherals.RNG)).unwrap();

    // Configure the system timer used by Embassy.
    let systimer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    // --- Watchdog (MWDT0) ---
    use esp_hal::timer::timg::MwdtStage;
    wdt.set_timeout(MwdtStage::Stage0, Duration::from_secs(30)); // reset after 30s if not fed
    wdt.enable();

    let wdt: &'static mut Mwdt0 = WDT_CELL.init(wdt);
    spawner.spawn(watchdog_task(wdt)).unwrap();

    // --- BLE controller for Trouble Host ---
    static INIT_CELL: StaticCell<EspWifiController<'static>> = StaticCell::new();
    let init = INIT_CELL.init(init);

    let connector = BleConnector::new(init, peripherals.BT);
    let controller: ExternalController<_, COMMAND_SLOTS> = ExternalController::new(connector);

    // Host resources (packet pool, L2CAP buffers, etc.)
    static HOST_RESOURCES: StaticCell<
        HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX>,
    > = StaticCell::new();
    let resources = HOST_RESOURCES.init(HostResources::new());

    // Build the controller
    static HOST_BUILDER_CELL: StaticCell<
        Stack<'static, ExternalController<BleConnector<'static>, COMMAND_SLOTS>, DefaultPacketPool>,
    > = StaticCell::new();
    let host_builder = HOST_BUILDER_CELL
        .init(trouble_host::new(controller, resources).set_random_address(ble_addr()));

    static STACK_CELL: StaticCell<
        Host<'static, ExternalController<BleConnector<'static>, COMMAND_SLOTS>, DefaultPacketPool>,
    > = StaticCell::new();
    let stack = STACK_CELL.init(host_builder.build());

    let &mut Host {
        ref mut peripheral,
        ref mut runner,
        ..
    } = stack;

    // Spawn the controller/host runner task.
    spawner.must_spawn(ble_runner_task(runner));

    // --- I²C (SDA=IO6, SCL=IO7) ---
    let i2c0 = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_hz(10_000)), // <= 15 kHz required by AGS10
    )
    .unwrap()
    .with_sda(peripherals.GPIO6)
    .with_scl(peripherals.GPIO7);

    let i2c_bus: Mutex<CriticalSectionRawMutex, RefCell<I2c<'_, esp_hal::Blocking>>> =
        Mutex::new(RefCell::new(i2c0));

    let mut delay = Delay;

    // Dedicated I2C device handles derived from the shared manager
    let ags10_i2c = I2cDevice::new(&i2c_bus);
    let aht20_i2c = I2cDevice::new(&i2c_bus);
    let bh1750_i2c = I2cDevice::new(&i2c_bus);
    let bmp280_i2c = I2cDevice::new(&i2c_bus);
    let scd40_i2c = I2cDevice::new(&i2c_bus);

    // AGS10
    let mut ags10 = Ags10::new(ags10_i2c);

    // AHT20
    let mut aht20 = AHT20::new(aht20_i2c, SENSOR_ADDRESS);
    let mut aht20 = aht20
        .init(&mut delay)
        .expect("AHT20 init/calibration failed");

    // BH1750
    let mut bh1750 = BH1750::new(bh1750_i2c, Delay, false);

    // BMP280 (BME280 driver)
    let mut bmp280 = BME280::new_secondary(bmp280_i2c);
    bmp280.init(&mut delay).expect("BMP280 init failed");

    // SCD40
    let mut scd40 = Scd4x::new(scd40_i2c, Delay);
    scd40.stop_periodic_measurement().ok();
    scd40
        .start_periodic_measurement()
        .expect("SCD40: start periodic failed");

    info!("Starting advertising and Environmental Sensing GATT service");
    let device_name = build_device_name();
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: device_name,
        appearance: &appearance::thermometer::GENERIC_THERMOMETER,
    }))
    .unwrap();

    loop {
        match advertise(device_name, peripheral, &server).await {
            Ok(conn) => {
                let cancel: Signal<CriticalSectionRawMutex, ()> = Signal::new();

                // GATT handling task
                let gatt_fut = async {
                    let r = gatt_events_task(&conn).await;
                    cancel.signal(());
                    r
                };

                // Periodic notifier task (10s)
                let notify_fut = async {
                    loop {
                        // Temperature & RH
                        let (temperature_c, humidity_rh): (f32, f32) =
                            match aht20.measure(&mut delay) {
                                Ok(m) => (m.temperature, m.humidity),
                                Err(e) => {
                                    warn!("[aht20] measure failed: {e:?}");
                                    Timer::after_millis(250).await;
                                    continue;
                                }
                            };

                        // CO₂
                        let mut co2_ppm: Option<u16> = None;
                        match scd40.data_ready_status() {
                            Ok(true) => match scd40.measurement() {
                                Ok(sample) => {
                                    co2_ppm = Some(clamp_u16(sample.co2));
                                }
                                Err(e) => {
                                    warn!("[scd40] read measurement failed: {e:?}");
                                }
                            },
                            Ok(false) => info!("[scd40] data not ready"),
                            Err(e) => warn!("[scd40] data_ready_status failed: {e:?}"),
                        }
                        let co2_ppm_u16 = co2_ppm.unwrap_or(0xFFFF);

                        // VOC
                        let voc_ppb_u16 = match ags10.read_tvoc() {
                            Ok((tvoc_ppb, _status)) => clamp_es_u16_from_u32(tvoc_ppb),
                            Err(e) => {
                                warn!("[ags10] read_tvoc failed: {e:?}");
                                0xFFFF
                            }
                        };

                        // Pressure
                        let pressure_d_pa_u32 = match bmp280.measure(&mut delay) {
                            Ok(m) => pa_to_es_decipascal(m.pressure),
                            Err(e) => {
                                warn!("[bmp280] measure failed: {e:?}");
                                0xFFFF_FFFF
                            }
                        };

                        // Illuminance
                        let (lux_u24_bytes, lux_for_log) =
                            match bh1750.get_one_time_measurement(Resolution::High) {
                                Ok(lux) => {
                                    let b = lux_to_es_u24_illuminance_bytes(
                                        lux / LIGHT_TRANSMISSION_FACTOR,
                                    );
                                    let n = if b == U24_UNKNOWN {
                                        None
                                    } else {
                                        Some(u24le_to_u32(b) as f32 / 100.0f32)
                                    };
                                    (b, n)
                                }
                                Err(e) => {
                                    warn!("[bh1750] measure failed: {e:?}");
                                    (U24_UNKNOWN, None)
                                }
                            };

                        info!(
                            "[conn] SCD40: {} ppm; AHT20: {:.2} °C, {:.2}%RH; AGS10: {} ppb; BMP280: {:.1} Pa; BH1750: {} lux",
                            co2_ppm_u16,
                            temperature_c,
                            humidity_rh,
                            voc_ppb_u16,
                            (pressure_d_pa_u32 as f32) / 10.0f32,
                            lux_for_log.map_or(0.0f32, |f| f)
                        );

                        if let Err(e) = server
                            .notify_environmental(
                                &conn,
                                temperature_c,
                                humidity_rh,
                                co2_ppm_u16,
                                voc_ppb_u16,
                                pressure_d_pa_u32,
                                lux_u24_bytes,
                            )
                            .await
                        {
                            warn!("[conn] notify failed: {e:?}");
                            break;
                        }

                        match select(Timer::after_millis(10_000), cancel.wait()).await {
                            Either::First(_) => {}
                            Either::Second(_) => break,
                        }
                    }
                    info!("[conn] notifier loop ended");
                };

                match select(gatt_fut, notify_fut).await {
                    Either::First(Ok(())) => info!("[conn] GATT ended"),
                    Either::First(Err(e)) => warn!("[conn] GATT error: {e:?}"),
                    Either::Second(()) => info!("[conn] Notifier ended"),
                }
            }
            Err(e) => {
                panic!("[adv] error: {:?}", e);
            }
        }
    }
}

fn build_device_name() -> &'static str {
    let addr = ble_addr();
    let bytes = addr.addr.raw();
    let s: String<32> = format!(
        "{}-{:02X}{:02X}{:02X}",
        ADVERTISED_NAME_PREFIX, bytes[2], bytes[1], bytes[0]
    )
    .unwrap();
    DEVICE_NAME.init(s).as_str()
}

fn ble_addr() -> Address {
    let mut addr = Efuse::read_base_mac_address();
    addr.reverse();
    Address::random(addr[..6].try_into().unwrap())
}

async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut adv_data = [0; 31];
    let adv_len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[[0x1A, 0x18]]),
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut adv_data[..],
    )?;

    let mut scan_data = [0; 31];
    let scan_len = AdStructure::encode_slice(
        &[AdStructure::CompleteLocalName(name.as_bytes())],
        &mut scan_data[..],
    )?;

    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..adv_len],
                scan_data: &scan_data[..scan_len],
            },
        )
        .await?;
    info!("[adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("[adv] connection established");
    Ok(conn)
}

// ----------------------------- GATT SERVER -----------------------------

#[gatt_server]
struct Server {
    pub environmental: EnvironmentalSensingService,
}

impl Server<'_> {
    pub async fn notify_environmental<P: PacketPool>(
        &self,
        conn: &GattConnection<'_, '_, P>,
        temperature_celsius: f32,
        humidity_percent: f32,
        co2_ppm: u16,
        voc_ppb: u16,
        pressure_deci_pa: u32,
        illuminance_u24: [u8; 3],
    ) -> Result<(), trouble_host::Error> {
        let temperature_es_centi = celsius_to_es_centi(temperature_celsius);
        let humidity_es_centi = rh_to_es_centi(humidity_percent);

        self.environmental
            .temperature
            .set(self, &temperature_es_centi)?;
        self.environmental.humidity.set(self, &humidity_es_centi)?;
        self.environmental.co2_concentration.set(self, &co2_ppm)?;
        self.environmental.voc_concentration.set(self, &voc_ppb)?;
        self.environmental.pressure.set(self, &pressure_deci_pa)?;
        self.environmental.illuminance.set(self, &illuminance_u24)?;

        self.environmental
            .temperature
            .notify(conn, &temperature_es_centi)
            .await?;
        self.environmental
            .humidity
            .notify(conn, &humidity_es_centi)
            .await?;
        self.environmental
            .co2_concentration
            .notify(conn, &co2_ppm)
            .await?;
        self.environmental
            .voc_concentration
            .notify(conn, &voc_ppb)
            .await?;
        self.environmental
            .pressure
            .notify(conn, &pressure_deci_pa)
            .await?;
        self.environmental
            .illuminance
            .notify(conn, &illuminance_u24)
            .await
    }
}

#[gatt_service(uuid = service::ENVIRONMENTAL_SENSING)]
struct EnvironmentalSensingService {
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, read, value = "Temperature °C")]
    #[characteristic(uuid = characteristic::TEMPERATURE, read, notify)]
    pub temperature: i16,

    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, read, value = "Relative Humidity %")]
    #[characteristic(uuid = characteristic::HUMIDITY, read, notify)]
    pub humidity: u16,

    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, read, value = "CO2 ppm")]
    #[characteristic(uuid = characteristic::CO2_CONCENTRATION, read, notify)]
    pub co2_concentration: u16,

    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, read, value = "VOC ppb")]
    #[characteristic(uuid = characteristic::VOC_CONCENTRATION, read, notify)]
    pub voc_concentration: u16,

    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, read, value = "Pressure Pa")]
    #[characteristic(uuid = characteristic::PRESSURE, read, notify)]
    pub pressure: u32,

    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, read, value = "Illuminance 0.01 lx")]
    #[characteristic(uuid = characteristic::ILLUMINANCE, read, notify)]
    pub illuminance: [u8; 3],
}

async fn gatt_events_task<P: PacketPool>(conn: &GattConnection<'_, '_, P>) -> Result<(), Error> {
    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            GattConnectionEvent::Gatt { event } => {
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => warn!("[gatt] response error: {e:?}"),
                };
            }
            _ => {}
        }
    };
    info!("[gatt] disconnected: {reason:?}");
    Ok(())
}

// ----------------------------- HELPERS -----------------------------

fn celsius_to_es_centi(c: f32) -> i16 {
    let mut centi = roundf(c * 100.0f32);
    centi = centi.clamp(-27315.0f32, 32767.0f32);
    centi as i16
}

fn rh_to_es_centi(rh_percent: f32) -> u16 {
    let mut centi = roundf(rh_percent * 100.0f32);
    centi = centi.clamp(0.0f32, 10000.0f32);
    centi as u16
}

fn clamp_u16(v: u16) -> u16 {
    if v == 0xFFFF { 0xFFFE } else { v.min(65533) }
}

fn clamp_es_u16_from_u32(v: u32) -> u16 {
    if v >= 0xFFFF {
        0xFFFE
    } else {
        (v as u16).min(65533)
    }
}

/// Convert Pa → Bluetooth ES Pressure (0x2A6D) deci-Pa (uint32, exponent -1).
fn pa_to_es_decipascal(pa: f32) -> u32 {
    let mut deci = roundf(pa * 10.0f32);
    deci = deci.clamp(0.0f32, u32::MAX as f32);
    deci as u32
}

/// Special value for Illuminance (uint24): unknown (0xFFFFFF).
const U24_UNKNOWN: [u8; 3] = [0xFF, 0xFF, 0xFF];

/// Convert BH1750 lux (f32) → ES "Illuminance" uint24 little-endian (0.01-lux resolution).
/// Clamps to [0, 0xFFFFFE]; 0xFFFFFF is reserved for 'unknown'.
fn lux_to_es_u24_illuminance_bytes(lux: f32) -> [u8; 3] {
    if !lux.is_finite() {
        return U24_UNKNOWN;
    }
    // scale: 0.01 lx resolution => value = round(lux * 100)
    let mut v = roundf(lux * 100.0f32);
    v = v.clamp(0.0f32, 16_777_214.0f32);
    let n = v as u32; // guaranteed <= 0xFFFFFE
    [
        (n & 0xFF) as u8,
        ((n >> 8) & 0xFF) as u8,
        ((n >> 16) & 0xFF) as u8,
    ]
}

/// Helper: convert uint24 little-endian bytes back to u32 (for logging only).
fn u24le_to_u32(b: [u8; 3]) -> u32 {
    (b[0] as u32) | ((b[1] as u32) << 8) | ((b[2] as u32) << 16)
}
