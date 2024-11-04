import yaml
import csv

class Database():
    """Class to store the raw data and generate the CSV file"""
    time_readings = []

    temperature_delta_time = []
    temperature_readings = []
    temperature_setpoint = []
    temperature_error = []
    temperature_pid_output = []
    temperature_kp = []
    temperature_ki = []
    temperature_kd = []
    extruder_rpm = []
    
    diameter_delta_time = []
    diameter_readings = []
    diameter_setpoint = []

    spooler_delta_time = []
    spooler_setpoint = []
    spooler_rpm = []
    spooler_gain = []
    spooler_oscilation_period = []

    fan_duty_cycle = []

    @classmethod
    def generate_csv(cls, filename: str) -> None:
        """Generate a CSV file with the data"""
        filename = filename + ".csv"
        with open(filename, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow(["Time (s)", "Temperature delta time (s)", 
                            "Temperature (C)", "Temperature setpoint (C)",
                            "Temperature error (C)", "Temperature PID output",
                            "Temperature Kp", "Temperature Ki", "Temperature Kd",
                            "Extruder RPM", "Diameter delta time (s)",
                            "Diameter (mm)", "Diameter setpoint (mm)",
                            "Spooler delta time (s)", "Spooler setpoint (RPM)",
                            "Spooler RPM", "Spooler gain", "Spooler oscilation period",
                            "Fan duty cycle (%)"])
            # Time array is bigger than the rest, make all arrays the same size
            max_length = max(len(cls.time_readings), len(cls.temperature_delta_time),
                            len(cls.temperature_readings), len(cls.temperature_setpoint),
                            len(cls.temperature_error), len(cls.temperature_pid_output),
                            len(cls.temperature_kp), len(cls.temperature_ki),
                            len(cls.temperature_kd), len(cls.extruder_rpm),
                            len(cls.diameter_delta_time), len(cls.diameter_readings),
                            len(cls.diameter_setpoint), len(cls.spooler_delta_time),
                            len(cls.spooler_setpoint), len(cls.spooler_rpm),
                            len(cls.spooler_gain), len(cls.spooler_oscilation_period),
                            len(cls.fan_duty_cycle))
            for i in range(max_length):
                row = [cls.time_readings[i] if i < len(cls.time_readings) else "",
                       cls.temperature_delta_time[i] if i < len(cls.temperature_delta_time) else "",
                       cls.temperature_readings[i] if i < len(cls.temperature_readings) else "",
                       cls.temperature_setpoint[i] if i < len(cls.temperature_setpoint) else "",
                       cls.temperature_error[i] if i < len(cls.temperature_error) else "",
                       cls.temperature_pid_output[i] if i < len(cls.temperature_pid_output) else "",
                       cls.temperature_kp[i] if i < len(cls.temperature_kp) else "",
                       cls.temperature_ki[i] if i < len(cls.temperature_ki) else "",
                       cls.temperature_kd[i] if i < len(cls.temperature_kd) else "",
                       cls.extruder_rpm[i] if i < len(cls.extruder_rpm) else "",
                       cls.diameter_delta_time[i] if i < len(cls.diameter_delta_time) else "",
                       cls.diameter_readings[i] if i < len(cls.diameter_readings) else "",
                       cls.diameter_setpoint[i] if i < len(cls.diameter_setpoint) else "",
                       cls.spooler_delta_time[i] if i < len(cls.spooler_delta_time) else "",
                       cls.spooler_setpoint[i] if i < len(cls.spooler_setpoint) else "",
                       cls.spooler_rpm[i] if i < len(cls.spooler_rpm) else "",
                       cls.spooler_gain[i] if i < len(cls.spooler_gain) else "",
                       cls.spooler_oscilation_period[i] if i < len(cls.spooler_oscilation_period) else "",
                       cls.fan_duty_cycle[i] if i < len(cls.fan_duty_cycle) else ""]
                writer.writerow(row)
        print(f"CSV file {filename} generated.")

    @staticmethod
    def get_calibration_data(field: str) -> float:
        """Get calibration data from the yaml file"""
        with open("calibration.yaml", "r", encoding="utf-8") as file:
            calibration_data = yaml.unsafe_load(file)
        return calibration_data[field]

    @staticmethod
    def update_calibration_data(field: str, value: str) -> None:
        """Update calibration data in the yaml file"""
        with open("calibration.yaml", "r") as file:
            calibration_data = yaml.unsafe_load(file)
        with open("calibration.yaml", "w") as file:
            calibration_data[field] = float(value)
            yaml.dump(calibration_data, file)
