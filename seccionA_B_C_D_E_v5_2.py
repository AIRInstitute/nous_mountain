"""
seccionA_B_C_D_E_v5_2.py
=========================
Version 5.2: catalogo de escenarios pre-definidos.

Cambio respecto a v5.1:
  - Los 12 escenarios del grid factorial estan definidos en la
    constante SCENARIOS al inicio del script.
  - Para correr un escenario, solo se cambia ACTIVE_SCENARIO al
    identificador deseado (p.ej. 'S05').
  - Todas las constantes SCENARIO_* se cargan automaticamente.

Diseno experimental:
  Factor 1: vehiculo (tesla_model3, audi_etron)
  Factor 2: SOC inicial (0.90, 0.40)
  Factor 3: clima (dry24, wet10, snow_neg5)
  Total: 2 x 2 x 3 = 12 escenarios

Opcion A: solo ida, fin por distancia recorrida acumulada.
"""

import carla
import csv
import queue
import time
import uuid
import cv2
import numpy as np
import math
from collections import deque

from ev_model import ElectricVehicleModel, get_vehicle_params


# ============================================================
#   CATALOGO DE ESCENARIOS  (no editar)
# ============================================================
# Cada entrada define un escenario completo del grid factorial 2x2x3.
# Para anadir un escenario nuevo, simplemente agrega una clave aqui.

SCENARIOS = {
    # ---------- Tesla Model 3, SOC 90% ----------
    'S01': {
        'id': 'S01_tesla_SOC90_dry24',
        'vehicle_key': 'tesla_model3',
        'start_soc': 0.90,
        't_ambient': 24.0,
        'road_condition': 'dry',
        'precipitation': 0.0,
        'wetness': 0.0,
        'wind_intensity': 0.0,
        'wind_dir_deg': 0.0,
        'cloudiness': 20.0,
        'sun_altitude': 30.0,
    },
    'S02': {
        'id': 'S02_tesla_SOC90_wet10',
        'vehicle_key': 'tesla_model3',
        'start_soc': 0.90,
        't_ambient': 10.0,
        'road_condition': 'wet',
        'precipitation': 60.0,
        'wetness': 70.0,
        'wind_intensity': 30.0,
        'wind_dir_deg': 180.0,
        'cloudiness': 90.0,
        'sun_altitude': 10.0,
    },
    'S03': {
        'id': 'S03_tesla_SOC90_snow_neg5',
        'vehicle_key': 'tesla_model3',
        'start_soc': 0.90,
        't_ambient': -5.0,
        'road_condition': 'snow',
        'precipitation': 80.0,
        'wetness': 40.0,
        'wind_intensity': 50.0,
        'wind_dir_deg': 180.0,
        'cloudiness': 100.0,
        'sun_altitude': 5.0,
    },
    # ---------- Tesla Model 3, SOC 40% ----------
    'S04': {
        'id': 'S04_tesla_SOC40_dry24',
        'vehicle_key': 'tesla_model3',
        'start_soc': 0.40,
        't_ambient': 24.0,
        'road_condition': 'dry',
        'precipitation': 0.0,
        'wetness': 0.0,
        'wind_intensity': 0.0,
        'wind_dir_deg': 0.0,
        'cloudiness': 20.0,
        'sun_altitude': 30.0,
    },
    'S05': {
        'id': 'S05_tesla_SOC40_wet10',
        'vehicle_key': 'tesla_model3',
        'start_soc': 0.40,
        't_ambient': 10.0,
        'road_condition': 'wet',
        'precipitation': 60.0,
        'wetness': 70.0,
        'wind_intensity': 30.0,
        'wind_dir_deg': 180.0,
        'cloudiness': 90.0,
        'sun_altitude': 10.0,
    },
    'S06': {
        'id': 'S06_tesla_SOC40_snow_neg5',
        'vehicle_key': 'tesla_model3',
        'start_soc': 0.40,
        't_ambient': -5.0,
        'road_condition': 'snow',
        'precipitation': 80.0,
        'wetness': 40.0,
        'wind_intensity': 50.0,
        'wind_dir_deg': 180.0,
        'cloudiness': 100.0,
        'sun_altitude': 5.0,
    },
    # ---------- Audi e-tron, SOC 90% ----------
    'S07': {
        'id': 'S07_audi_SOC90_dry24',
        'vehicle_key': 'audi_etron',
        'start_soc': 0.90,
        't_ambient': 24.0,
        'road_condition': 'dry',
        'precipitation': 0.0,
        'wetness': 0.0,
        'wind_intensity': 0.0,
        'wind_dir_deg': 0.0,
        'cloudiness': 20.0,
        'sun_altitude': 30.0,
    },
    'S08': {
        'id': 'S08_audi_SOC90_wet10',
        'vehicle_key': 'audi_etron',
        'start_soc': 0.90,
        't_ambient': 10.0,
        'road_condition': 'wet',
        'precipitation': 60.0,
        'wetness': 70.0,
        'wind_intensity': 30.0,
        'wind_dir_deg': 180.0,
        'cloudiness': 90.0,
        'sun_altitude': 10.0,
    },
    'S09': {
        'id': 'S09_audi_SOC90_snow_neg5',
        'vehicle_key': 'audi_etron',
        'start_soc': 0.90,
        't_ambient': -5.0,
        'road_condition': 'snow',
        'precipitation': 80.0,
        'wetness': 40.0,
        'wind_intensity': 50.0,
        'wind_dir_deg': 180.0,
        'cloudiness': 100.0,
        'sun_altitude': 5.0,
    },
    # ---------- Audi e-tron, SOC 40% ----------
    'S10': {
        'id': 'S10_audi_SOC40_dry24',
        'vehicle_key': 'audi_etron',
        'start_soc': 0.40,
        't_ambient': 24.0,
        'road_condition': 'dry',
        'precipitation': 0.0,
        'wetness': 0.0,
        'wind_intensity': 0.0,
        'wind_dir_deg': 0.0,
        'cloudiness': 20.0,
        'sun_altitude': 30.0,
    },
    'S11': {
        'id': 'S11_audi_SOC40_wet10',
        'vehicle_key': 'audi_etron',
        'start_soc': 0.40,
        't_ambient': 10.0,
        'road_condition': 'wet',
        'precipitation': 60.0,
        'wetness': 70.0,
        'wind_intensity': 30.0,
        'wind_dir_deg': 180.0,
        'cloudiness': 90.0,
        'sun_altitude': 10.0,
    },
    'S12': {
        'id': 'S12_audi_SOC40_snow_neg5',
        'vehicle_key': 'audi_etron',
        'start_soc': 0.40,
        't_ambient': -5.0,
        'road_condition': 'snow',
        'precipitation': 80.0,
        'wetness': 40.0,
        'wind_intensity': 50.0,
        'wind_dir_deg': 180.0,
        'cloudiness': 100.0,
        'sun_altitude': 5.0,
    },
    # ---------- Tesla Cybertruck, SOC 90% ----------
    'S13': {
        'id': 'S13_cyber_SOC90_dry24',
        'vehicle_key': 'cybertruck',
        'start_soc': 0.90,
        't_ambient': 24.0,
        'road_condition': 'dry',
        'precipitation': 0.0,
        'wetness': 0.0,
        'wind_intensity': 0.0,
        'wind_dir_deg': 0.0,
        'cloudiness': 20.0,
        'sun_altitude': 30.0,
    },
    'S14': {
        'id': 'S14_cyber_SOC90_wet10',
        'vehicle_key': 'cybertruck',
        'start_soc': 0.90,
        't_ambient': 10.0,
        'road_condition': 'wet',
        'precipitation': 60.0,
        'wetness': 70.0,
        'wind_intensity': 30.0,
        'wind_dir_deg': 180.0,
        'cloudiness': 90.0,
        'sun_altitude': 10.0,
    },
    'S15': {
        'id': 'S15_cyber_SOC90_snow_neg5',
        'vehicle_key': 'cybertruck',
        'start_soc': 0.90,
        't_ambient': -5.0,
        'road_condition': 'snow',
        'precipitation': 80.0,
        'wetness': 40.0,
        'wind_intensity': 50.0,
        'wind_dir_deg': 180.0,
        'cloudiness': 100.0,
        'sun_altitude': 5.0,
    },
    # ---------- Tesla Cybertruck, SOC 40% ----------
    'S16': {
        'id': 'S16_cyber_SOC40_dry24',
        'vehicle_key': 'cybertruck',
        'start_soc': 0.40,
        't_ambient': 24.0,
        'road_condition': 'dry',
        'precipitation': 0.0,
        'wetness': 0.0,
        'wind_intensity': 0.0,
        'wind_dir_deg': 0.0,
        'cloudiness': 20.0,
        'sun_altitude': 30.0,
    },
    'S17': {
        'id': 'S17_cyber_SOC40_wet10',
        'vehicle_key': 'cybertruck',
        'start_soc': 0.40,
        't_ambient': 10.0,
        'road_condition': 'wet',
        'precipitation': 60.0,
        'wetness': 70.0,
        'wind_intensity': 30.0,
        'wind_dir_deg': 180.0,
        'cloudiness': 90.0,
        'sun_altitude': 10.0,
    },
    'S18': {
        'id': 'S18_cyber_SOC40_snow_neg5',
        'vehicle_key': 'cybertruck',
        'start_soc': 0.40,
        't_ambient': -5.0,
        'road_condition': 'snow',
        'precipitation': 80.0,
        'wetness': 40.0,
        'wind_intensity': 50.0,
        'wind_dir_deg': 180.0,
        'cloudiness': 100.0,
        'sun_altitude': 5.0,
    },

}


# ============================================================
#   SELECCION DEL ESCENARIO   <-- LO UNICO QUE CAMBIAS ENTRE RUNS
# ============================================================

ACTIVE_SCENARIO = 'S11'   # cambia a 'S02', 'S03', ..., 'S12' segun la run


# ============================================================
#   CARGA AUTOMATICA DEL ESCENARIO ACTIVO
# ============================================================

if ACTIVE_SCENARIO not in SCENARIOS:
    raise ValueError(
        f"ACTIVE_SCENARIO='{ACTIVE_SCENARIO}' no esta en SCENARIOS. "
        f"Opciones validas: {list(SCENARIOS.keys())}"
    )

_s = SCENARIOS[ACTIVE_SCENARIO]
SCENARIO_ID = _s['id']
SCENARIO_VEHICLE_KEY = _s['vehicle_key']
SCENARIO_START_SOC = _s['start_soc']
SCENARIO_T_AMBIENT = _s['t_ambient']
SCENARIO_ROAD_CONDITION = _s['road_condition']
SCENARIO_PRECIPITATION = _s['precipitation']
SCENARIO_WETNESS = _s['wetness']
SCENARIO_WIND_INTENSITY = _s['wind_intensity']
SCENARIO_WIND_DIR_DEG = _s['wind_dir_deg']
SCENARIO_CLOUDINESS = _s['cloudiness']
SCENARIO_SUN_ALTITUDE = _s['sun_altitude']
SCENARIO_SUN_AZIMUTH = 270.0

# Flags globales (afectan a todos los escenarios)
ENABLE_AUTO_LIGHTS = True
ENABLE_CLIMATE_COUPLING = True


# ============================================================
#   CONFIGURACION TECNICA
# ============================================================

OUTPUT_FILE = f'datos_grid_{SCENARIO_ID}.csv'

# Parametros de fin de trayecto: distancia recorrida acumulada.
# Medido empiricamente: del spawn_points[0] hasta la rotonda alta del puerto
# el coche recorre aproximadamente 10.5-11 km de carretera serpenteante.
# Con umbral 10000 m la run cierra al llegar al extremo, dejando algo de
# margen sobre la medicion real de 10887 m de la run de referencia.
DEST_TRAVELED_DISTANCE_M = 10000.0      # umbral de distancia recorrida acumulada
SIMULATION_DURATION_MAX = 3600.0        # 60 min de seguridad (red final)

# Detector de atasco
STUCK_WINDOW_S = 30.0
STUCK_MIN_DISTANCE_M = 10.0             # progreso minimo en la ventana

# Diagnostico
RATE_REPORT_INTERVAL_S = 30.0

VISUALIZE_CAMERA = True
MAP_NAME = 'Mountain'
SPAWN_Z_OFFSET = 0.5


# ============================================================
#   UTILIDADES
# ============================================================

def normalize_pitch(pitch_deg):
    """Lleva un angulo de pitch al rango fisico [-90, 90]."""
    pitch = ((pitch_deg + 180.0) % 360.0) - 180.0
    if pitch > 90.0:
        pitch = 180.0 - pitch
    elif pitch < -90.0:
        pitch = -180.0 - pitch
    return pitch


def compute_wind_long(wind_speed_ms, wind_dir_deg, vehicle_yaw_deg):
    """Componente longitudinal del viento. + = viento de cara."""
    wind_rad = math.radians(wind_dir_deg)
    wind_x = wind_speed_ms * math.cos(wind_rad)
    wind_y = wind_speed_ms * math.sin(wind_rad)
    yaw_rad = math.radians(vehicle_yaw_deg)
    fwd_x = math.cos(yaw_rad)
    fwd_y = math.sin(yaw_rad)
    return -(wind_x * fwd_x + wind_y * fwd_y)


# ============================================================
#   MAIN
# ============================================================

def main():
    vehicle_params = get_vehicle_params(SCENARIO_VEHICLE_KEY)
    vehicle_blueprint = vehicle_params['carla_blueprint']
    vehicle_display = vehicle_params['display_name']

    print("=" * 80)
    print(f"CARLA SCRIPT v5 - Escenario: {SCENARIO_ID}")
    print("=" * 80)
    print(f"  Vehiculo:         {vehicle_display}")
    print(f"  Masa:             {vehicle_params['mass_curb']} kg")
    print(f"  Bateria util:     {vehicle_params['usable_kwh']} kWh")
    print(f"  SOC inicial:      {SCENARIO_START_SOC*100:.0f}%")
    print(f"  T ambiente:       {SCENARIO_T_AMBIENT}°C")
    print(f"  Pavimento:        {SCENARIO_ROAD_CONDITION}")
    print(f"  Precipitacion:    {SCENARIO_PRECIPITATION:.0f}/100")
    print(f"  Sol:              alt={SCENARIO_SUN_ALTITUDE}° azim={SCENARIO_SUN_AZIMUTH}°")
    print(f"  Acople climatico: {ENABLE_CLIMATE_COUPLING}")
    print(f"  Luces auto:       {ENABLE_AUTO_LIGHTS}")
    print(f"  Archivo salida:   {OUTPUT_FILE}")
    print()

    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)

    print(f"Cargando mapa '{MAP_NAME}'...")
    world = client.load_world(MAP_NAME)
    m = world.get_map()
    print(f"Mapa cargado: {m.name}")

    weather = carla.WeatherParameters(
        cloudiness=SCENARIO_CLOUDINESS,
        precipitation=SCENARIO_PRECIPITATION,
        wetness=SCENARIO_WETNESS,
        wind_intensity=SCENARIO_WIND_INTENSITY,
        sun_altitude_angle=SCENARIO_SUN_ALTITUDE,
        sun_azimuth_angle=SCENARIO_SUN_AZIMUTH,
    )
    world.set_weather(weather)

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    settings_actuales = world.get_settings()
    if not settings_actuales.synchronous_mode:
        print("AVISO: el servidor NO esta en modo sincrono.")
    else:
        print(f"Modo sincrono confirmado, fixed_delta = "
              f"{settings_actuales.fixed_delta_seconds}s")

    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)
    traffic_manager.global_percentage_speed_difference(0.0)

    # === SPAWN ===
    # Igual que el v2 original: usar el primer spawn point del HD Map.
    spawn_points = m.get_spawn_points()
    if not spawn_points:
        raise RuntimeError("El mapa no tiene spawn points definidos.")
    spawn_origin = spawn_points[0]
    origin_x = spawn_origin.location.x
    origin_y = spawn_origin.location.y
    print(f"Spawn en spawn_points[0]: ({origin_x:.1f}, {origin_y:.1f}, "
          f"{spawn_origin.location.z:.1f})")
    print(f"Fin de trayecto: cuando la distancia recorrida acumulada >= "
          f"{DEST_TRAVELED_DISTANCE_M:.0f} m.")

    # Spectator: vista comoda sobre el spawn
    spectator = world.get_spectator()
    spec_loc = carla.Location(
        x=origin_x, y=origin_y, z=spawn_origin.location.z + 30.0
    )
    spectator.set_transform(carla.Transform(spec_loc, carla.Rotation(pitch=-25.0)))

    run_uuid = str(uuid.uuid4())[:8]

    ev = ElectricVehicleModel(
        params=vehicle_params,
        start_soc=SCENARIO_START_SOC,
        t_ambient_c=SCENARIO_T_AMBIENT,
        road_condition=SCENARIO_ROAD_CONDITION,
        enable_climate_coupling=ENABLE_CLIMATE_COUPLING,
    )

    actor_list = []
    image_queue = queue.Queue()

    # Variables que el bloque finally necesita en cualquier flujo
    elec = None
    current_relative_time = 0.0
    stop_reason = "max_duration"
    dist_recorrida_total_m = 0.0

    try:
        bp_lib = world.get_blueprint_library()
        vehicle_bp = bp_lib.find(vehicle_blueprint)

        spawn_transform = spawn_origin
        spawn_transform.location.z += SPAWN_Z_OFFSET
        vehicle = world.spawn_actor(vehicle_bp, spawn_transform)
        actor_list.append(vehicle)

        phy_ctrl = vehicle.get_physics_control()
        phy_ctrl.mass = vehicle_params['mass_curb']
        vehicle.apply_physics_control(phy_ctrl)

        vehicle.set_autopilot(True)

        # Luces automaticas via Traffic Manager
        if ENABLE_AUTO_LIGHTS:
            try:
                traffic_manager.update_vehicle_lights(vehicle, True)
                print("Luces automaticas activadas (gestionadas por el TM).")
            except AttributeError:
                if SCENARIO_SUN_ALTITUDE < 0:
                    light_state = (carla.VehicleLightState.LowBeam |
                                   carla.VehicleLightState.Position)
                    vehicle.set_light_state(carla.VehicleLightState(light_state))
                    print("Luces de cruce activadas manualmente (fallback).")

        # Camara RGB para visualizacion
        camera_bp = bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '640')
        camera_bp.set_attribute('image_size_y', '480')
        camera = world.spawn_actor(
            camera_bp,
            carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=-8.0)),
            attach_to=vehicle
        )
        actor_list.append(camera)
        camera.listen(image_queue.put)

        with open(OUTPUT_FILE, mode='w', newline='') as file:
            writer = csv.writer(file)

            header = [
                'scenario_id', 'vehicle_key', 'run_id', 'vehicle_id',
                'timestamp_abs', 'time_relative', 'dt', 'road_id', 'lane_id',
                'pos_x', 'pos_y', 'pos_z', 'heading',
                'road_grade_deg', 'road_curvature', 'road_friction',
                'speed_kmh', 'acc_long', 'acc_lat',
                'throttle', 'brake', 'steering', 'gear',
                'soc_percent', 'capacity_usable_wh', 'voltage_v', 'current_a',
                'power_watts', 'energy_used_cum_wh', 'energy_regen_cum_wh',
                'is_regen', 'battery_temp_c',
                'aux_power_w', 'c_rr_eff',
                'ambient_temp_c', 'precip_type', 'precip_intensity',
                'wind_speed_ms', 'wind_dir_deg', 'wind_long_ms',
                'road_condition'
            ]
            writer.writerow(header)

            world.tick()
            start_sim_time = world.get_snapshot().timestamp.elapsed_seconds
            wall_start = time.time()

            print(f"Iniciando Run {run_uuid}...")
            print()

            # Estado de control
            prev_pos = None
            progress_history = deque()           # ventana movil para detector atasco
            last_rate_report = 0.0
            last_frame_rgb = None

            while current_relative_time < SIMULATION_DURATION_MAX:
                world.tick()

                snapshot = world.get_snapshot()
                dt = snapshot.timestamp.delta_seconds
                current_relative_time = snapshot.timestamp.elapsed_seconds - start_sim_time

                vel = vehicle.get_velocity()
                speed_ms = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

                acc = vehicle.get_acceleration()
                transform = vehicle.get_transform()
                fwd = transform.get_forward_vector()
                acc_long = (acc.x * fwd.x) + (acc.y * fwd.y) + (acc.z * fwd.z)
                right = transform.get_right_vector()
                acc_lat = (acc.x * right.x) + (acc.y * right.y) + (acc.z * right.z)

                control = vehicle.get_control()
                loc = transform.location
                wp = m.get_waypoint(loc)

                # Acumular distancia recorrida (2D)
                if prev_pos is not None:
                    dist_recorrida_total_m += math.sqrt(
                        (loc.x - prev_pos[0]) ** 2 +
                        (loc.y - prev_pos[1]) ** 2
                    )
                prev_pos = (loc.x, loc.y)

                # Distancia al spawn (2D) -- solo para visualizacion/diagnostico,
                # NO se usa como criterio de fin (no monotoniza en trayectos
                # serpenteantes de montana).
                dist_from_origin = math.hypot(loc.x - origin_x, loc.y - origin_y)

                # --- CONDICION 1: fin de trayecto por distancia recorrida ---
                # La distancia FISICAMENTE recorrida si monotoniza con el avance
                # del coche. Cuando supera el umbral, sabemos que ha llegado al
                # final del trayecto definido.
                if dist_recorrida_total_m >= DEST_TRAVELED_DISTANCE_M and current_relative_time > 30.0:
                    stop_reason = "reached_destination"
                    print(f"\n[OK] Distancia objetivo alcanzada en "
                          f"t={current_relative_time:.1f}s "
                          f"(recorrido = {dist_recorrida_total_m:.0f} m)")
                    break

                # --- CONDICION 2: detector de atasco ---
                # Por distancia FISICAMENTE RECORRIDA en una ventana movil.
                # No depende de la direccion del avance.
                progress_history.append((current_relative_time, dist_recorrida_total_m))
                while (progress_history and
                       progress_history[0][0] < current_relative_time - STUCK_WINDOW_S):
                    progress_history.popleft()

                if current_relative_time > STUCK_WINDOW_S + 5.0:
                    t_old, d_old = progress_history[0]
                    progreso_ventana = dist_recorrida_total_m - d_old
                    if progreso_ventana < STUCK_MIN_DISTANCE_M:
                        stop_reason = "stuck"
                        print(f"\n[!] Vehiculo atascado en t={current_relative_time:.1f}s")
                        print(f"    Recorrio solo {progreso_ventana:.1f} m "
                              f"en los ultimos {STUCK_WINDOW_S:.0f}s")
                        break

                # --- Diagnostico de ritmo ---
                if current_relative_time - last_rate_report >= RATE_REPORT_INTERVAL_S:
                    wall_elapsed = time.time() - wall_start
                    if wall_elapsed > 0.1:
                        rate = current_relative_time / wall_elapsed
                        v_med = (dist_recorrida_total_m / current_relative_time) * 3.6 \
                                if current_relative_time > 0 else 0.0
                        print(f"  [t={current_relative_time:.0f}s wall={wall_elapsed:.0f}s] "
                              f"sim/wall={rate:.2f}x  v_med={v_med:.1f} km/h  "
                              f"dist_rec={dist_recorrida_total_m:.0f} m  "
                              f"dist_origen={dist_from_origin:.0f} m  "
                              f"SOC={ev.soc*100:.1f}%  z={loc.z:.0f}m")
                        if rate < 0.7:
                            print(f"    AVISO: simulacion lenta (deberia ir a 1.0x). "
                                  f"Considera --RenderOffScreen.")
                    last_rate_report = current_relative_time

                # --- Pendiente desde el VEHICULO ---
                # El HD Map tiene OpenDRIVE plano (z=0 en waypoints) pero el
                # mesh fisico si tiene relieve. El pitch del coche refleja
                # correctamente la pendiente del terreno.
                road_pitch_deg = normalize_pitch(transform.rotation.pitch)
                pitch_rad = math.radians(road_pitch_deg)

                ang_vel = vehicle.get_angular_velocity()
                curvature = math.radians(ang_vel.z) / speed_ms if speed_ms > 1 else 0

                wind_speed_ms_real = (SCENARIO_WIND_INTENSITY / 100.0) * 20.0
                wind_long = compute_wind_long(
                    wind_speed_ms_real, SCENARIO_WIND_DIR_DEG,
                    transform.rotation.yaw
                )

                elec = ev.update(
                    speed_ms, acc_long, pitch_rad, dt,
                    wind_long_ms=wind_long,
                )

                # --- Visualizacion no-bloqueante ---
                if VISUALIZE_CAMERA:
                    try:
                        nuevo_frame = None
                        while not image_queue.empty():
                            nuevo_frame = image_queue.get_nowait()

                        if nuevo_frame is not None:
                            array = np.frombuffer(nuevo_frame.raw_data, dtype=np.dtype("uint8"))
                            last_frame_rgb = np.reshape(
                                array, (nuevo_frame.height, nuevo_frame.width, 4)
                            )[:, :, :3].copy()

                        if last_frame_rgb is not None:
                            im_show = last_frame_rgb.copy()
                            cv2.putText(im_show, f"[{SCENARIO_ID}]", (10, 30),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 220, 255), 1)
                            cv2.putText(im_show, f"t: {current_relative_time:.1f}s", (10, 65),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            cv2.putText(im_show, f"SOC: {elec['soc']:.1f}%", (10, 100),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            cv2.putText(im_show, f"P: {elec['power_watts']/1000:.1f} kW", (10, 135),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            cv2.putText(im_show, f"v: {speed_ms*3.6:.1f} km/h", (10, 170),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            cv2.putText(im_show, f"grade: {road_pitch_deg:+.1f} deg", (10, 205),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                            cv2.putText(im_show, f"z: {loc.z:.1f} m", (10, 240),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                            cv2.putText(im_show, f"T_bat: {elec['temp']:.1f}C", (10, 280),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
                            cv2.putText(im_show, f"aux: {elec['aux_power_w']:.0f}W", (10, 315),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
                            cv2.putText(im_show,
                                        f"recorrido: {dist_recorrida_total_m:.0f}/{DEST_TRAVELED_DISTANCE_M:.0f} m",
                                        (10, 350),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

                            # Indicador visual del progreso en la ventana de atasco
                            if len(progress_history) > 1:
                                t_old, d_old = progress_history[0]
                                progreso_v = dist_recorrida_total_m - d_old
                                ventana_real = current_relative_time - t_old
                                if ventana_real > 5.0:
                                    color = (0, 255, 0) if progreso_v > STUCK_MIN_DISTANCE_M \
                                            else (0, 100, 255)
                                    cv2.putText(im_show,
                                                f"prog {ventana_real:.0f}s: {progreso_v:.0f}m "
                                                f"(min {STUCK_MIN_DISTANCE_M:.0f})",
                                                (10, 380),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.50, color, 1)

                            cv2.imshow("CARLA", im_show)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                stop_reason = "user_quit"
                                break
                    except queue.Empty:
                        pass
                    except Exception as e:
                        print(f"Error visualizacion: {e}")

                precip_type = "rain" if SCENARIO_PRECIPITATION > 0 else "none"

                row = [
                    SCENARIO_ID, SCENARIO_VEHICLE_KEY, run_uuid, vehicle.id,
                    round(snapshot.timestamp.elapsed_seconds, 4),
                    round(current_relative_time, 4), dt,
                    wp.road_id if wp else -1, wp.lane_id if wp else 0,
                    round(loc.x, 3), round(loc.y, 3), round(loc.z, 3),
                    round(transform.rotation.yaw, 3),
                    round(road_pitch_deg, 3), round(curvature, 5),
                    round(phy_ctrl.wheels[0].tire_friction, 2),
                    round(speed_ms * 3.6, 2), round(acc_long, 3), round(acc_lat, 3),
                    round(control.throttle, 3), round(control.brake, 3),
                    round(control.steer, 3), control.gear,
                    elec['soc'], elec['capacity_usable'], elec['voltage'],
                    elec['current'], elec['power_watts'],
                    elec['energy_used'], elec['energy_regen'],
                    elec['is_regen'], elec['temp'],
                    elec['aux_power_w'], elec['c_rr_eff'],
                    SCENARIO_T_AMBIENT, precip_type, SCENARIO_PRECIPITATION,
                    round(wind_speed_ms_real, 2), SCENARIO_WIND_DIR_DEG,
                    round(wind_long, 2), SCENARIO_ROAD_CONDITION
                ]
                writer.writerow(row)

            # --- REPORTE FINAL ---
            print()
            print("=" * 70)
            print(f"FIN DE RUN  ({stop_reason})")
            print("=" * 70)
            print(f"  Run ID:               {run_uuid}")
            print(f"  Duracion:             {current_relative_time:.1f} s "
                  f"({current_relative_time/60:.1f} min)")
            print(f"  Distancia recorrida:  {dist_recorrida_total_m:.0f} m "
                  f"({dist_recorrida_total_m/1000:.2f} km)")
            if elec is not None:
                print(f"  SOC final:            {elec['soc']:.2f}%  "
                      f"(caida: {SCENARIO_START_SOC*100 - elec['soc']:.2f}%)")
                print(f"  Energia consumida:    {elec['energy_used']:.0f} Wh")
                print(f"  Energia regenerada:   {elec['energy_regen']:.0f} Wh")
                print(f"  Energia neta:         {elec['energy_used']-elec['energy_regen']:.0f} Wh")
                print(f"  T bateria final:      {elec['temp']:.1f}°C")
            if stop_reason == "stuck":
                print()
                print("  AVISO: la run aborto por atasco. Revisa el video o el HUD")
                print("  antes de seguir con el resto del grid.")
            print()

    finally:
        try:
            settings.synchronous_mode = False
            world.apply_settings(settings)
        except Exception:
            pass
        for actor in actor_list:
            try:
                actor.destroy()
            except Exception:
                pass
        cv2.destroyAllWindows()
        print(f"Datos guardados en {OUTPUT_FILE}")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
