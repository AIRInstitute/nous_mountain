"""
ev_model.py
============
Modelo energetico modular con biblioteca de vehiculos parametrizados.
"""

import math


EV_LIBRARY = {
    'tesla_model3': {
        'display_name': 'Tesla Model 3 RWD',
        'carla_blueprint': 'vehicle.tesla.model3',
        'mass_curb': 1611,
        'Cd': 0.23,
        'A_front': 2.22,
        'C_rr': 0.011,
        'battery_kwh': 75.0,
        'usable_kwh': 72.0,
        'voltage_nom': 350.0,
        'eta_motor': 0.90,
        'eta_regen': 0.85,
        'aux_power_base': 300.0,
        'max_regen_power': 60000.0,
        'max_discharge_power': 210000.0,
        'r_internal_ohm': 0.07,
        'thermal_mass_J_per_K': 180000.0,
        'cooling_W_per_K': 250.0,
    },
    'audi_etron': {
        'display_name': 'Audi e-tron 55 quattro',
        'carla_blueprint': 'vehicle.audi.etron',
        'mass_curb': 2490,
        'Cd': 0.27,
        'A_front': 2.65,
        'C_rr': 0.012,
        'battery_kwh': 95.0,
        'usable_kwh': 86.5,
        'voltage_nom': 396.0,
        'eta_motor': 0.88,
        'eta_regen': 0.82,
        'aux_power_base': 400.0,
        'max_regen_power': 220000.0,
        'max_discharge_power': 300000.0,
        'r_internal_ohm': 0.06,
        'thermal_mass_J_per_K': 230000.0,
        'cooling_W_per_K': 320.0,
    },
    'cybertruck': {
        'display_name': 'Tesla Cybertruck Dual Motor AWD',
        'carla_blueprint': 'vehicle.tesla.cybertruck',
        'mass_curb': 3104,
        'Cd': 0.34,
        'A_front': 3.10,
        'C_rr': 0.013,
        'battery_kwh': 123.0,
        'usable_kwh': 118.0,
        'voltage_nom': 800.0,
        'eta_motor': 0.89,
        'eta_regen': 0.83,
        'aux_power_base': 450.0,
        'max_regen_power': 300000.0,
        'max_discharge_power': 450000.0,
        'r_internal_ohm': 0.05,
        'thermal_mass_J_per_K': 280000.0,
        'cooling_W_per_K': 380.0,
    },
}


def get_vehicle_params(vehicle_key):
    """Devuelve copia del diccionario de parametros del vehiculo solicitado."""
    if vehicle_key not in EV_LIBRARY:
        opciones = ', '.join(EV_LIBRARY.keys())
        raise KeyError(
            f"Vehiculo '{vehicle_key}' no encontrado. Opciones: {opciones}"
        )
    return EV_LIBRARY[vehicle_key].copy()


# Alias retrocompatibles
EV_PARAMS_TESLA_M3 = EV_LIBRARY['tesla_model3']
EV_PARAMS_SUV = EV_LIBRARY['audi_etron']


def hvac_aux_power(t_amb_c):
    """Potencia auxiliar HVAC (W) vs T_ambiente."""
    base = 300.0
    if 18.0 <= t_amb_c <= 22.0:
        return base
    if t_amb_c < 18.0:
        return base + 150.0 * (18.0 - t_amb_c)
    return base + 100.0 * (t_amb_c - 22.0)


def battery_capacity_factor(t_amb_c):
    """Factor multiplicativo (0..1) sobre capacidad utilizable vs T_ambiente."""
    if t_amb_c >= 20.0:
        return 1.0
    if t_amb_c >= 0.0:
        return 1.0 - 0.005 * (20.0 - t_amb_c)
    return 0.90 - 0.0075 * (-t_amb_c)


def rolling_coefficient(c_rr_base, road_condition):
    """Ajusta C_rr segun condicion de pavimento."""
    multipliers = {
        'dry': 1.00, 'wet': 1.20, 'puddles': 1.45,
        'snow': 2.00, 'ice': 1.50,
    }
    return c_rr_base * multipliers.get(road_condition, 1.0)


class ElectricVehicleModel:
    """Modelo longitudinal de VE con acoplamiento climatico opcional."""

    def __init__(self, params=None, start_soc=0.90, t_ambient_c=25.0,
                 road_condition='dry', enable_climate_coupling=True):
        if params is None:
            params = EV_LIBRARY['tesla_model3'].copy()
        self.p = params
        self.enable_climate_coupling = enable_climate_coupling
        self.t_ambient_c = t_ambient_c
        self.road_condition = road_condition

        if enable_climate_coupling:
            self.capacity_factor = battery_capacity_factor(t_ambient_c)
        else:
            self.capacity_factor = 1.0
        self.capacity_wh_nominal = self.p['usable_kwh'] * 1000.0
        self.capacity_wh = self.capacity_wh_nominal * self.capacity_factor

        self.soc = start_soc
        self.current_energy_wh = self.capacity_wh * self.soc
        self.energy_used_cum = 0.0
        self.energy_regen_cum = 0.0
        self.temp_battery = t_ambient_c

    def update(self, speed_ms, acc_long_ms2, road_grade_rad, dt,
               wind_long_ms=0.0, t_ambient_c=None, road_condition=None):
        if t_ambient_c is None:
            t_ambient_c = self.t_ambient_c
        if road_condition is None:
            road_condition = self.road_condition

        g = 9.81
        rho_air = 1.225

        if self.enable_climate_coupling:
            c_rr_eff = rolling_coefficient(self.p['C_rr'], road_condition)
            aux_power = hvac_aux_power(t_ambient_c)
            v_relative = speed_ms + wind_long_ms
        else:
            c_rr_eff = self.p['C_rr']
            aux_power = self.p['aux_power_base']
            v_relative = speed_ms

        f_grade = self.p['mass_curb'] * g * math.sin(road_grade_rad)
        f_roll = c_rr_eff * self.p['mass_curb'] * g * math.cos(road_grade_rad)
        f_aero = 0.5 * rho_air * self.p['Cd'] * self.p['A_front'] * (v_relative ** 2)
        if v_relative < 0:
            f_aero = -f_aero
        f_acc = self.p['mass_curb'] * acc_long_ms2

        f_total = f_roll + f_aero + f_grade + f_acc
        p_wheels = f_total * speed_ms

        if p_wheels > 0:
            p_battery = (p_wheels / self.p['eta_motor']) + aux_power
            p_battery = min(p_battery, self.p['max_discharge_power'])
        else:
            p_battery = (p_wheels * self.p['eta_regen']) + aux_power
            regen_limit = -self.p['max_regen_power'] if self.soc < 0.98 else 0.0
            p_battery = max(p_battery, regen_limit)

        energy_step_wh = (p_battery * dt) / 3600.0
        self.current_energy_wh -= energy_step_wh
        self.soc = max(0.0, min(1.0, self.current_energy_wh / self.capacity_wh))

        if energy_step_wh > 0:
            self.energy_used_cum += energy_step_wh
        else:
            self.energy_regen_cum += abs(energy_step_wh)

        v_min = self.p['voltage_nom'] * 0.86
        v_max = self.p['voltage_nom'] * 1.14
        voltage = v_min + (v_max - v_min) * self.soc
        current = p_battery / voltage if voltage > 0 else 0.0

        heat_gen_w = (current ** 2) * self.p['r_internal_ohm']
        cooling_w = (self.temp_battery - t_ambient_c) * self.p['cooling_W_per_K']
        dT = (heat_gen_w - cooling_w) * dt / self.p['thermal_mass_J_per_K']
        self.temp_battery += dT

        return {
            'soc': round(self.soc * 100, 3),
            'capacity_usable': round(self.capacity_wh, 1),
            'voltage': round(voltage, 2),
            'current': round(current, 2),
            'power_watts': round(p_battery, 2),
            'energy_used': round(self.energy_used_cum, 4),
            'energy_regen': round(self.energy_regen_cum, 4),
            'is_regen': 1 if p_battery < 0 else 0,
            'temp': round(self.temp_battery, 3),
            'aux_power_w': round(aux_power, 1),
            'c_rr_eff': round(c_rr_eff, 5),
            'limit_discharge': self.p['max_discharge_power'],
            'limit_regen': self.p['max_regen_power']
        }
