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
        
            # Obtener el tiempo total de ejecución
            total_time = cls.time_readings[-1] if cls.time_readings else 0
            
            # Temperature Table
            writer.writerow(["TEMPERATURE DATA"])
            writer.writerow(["Elapsed Time (s)", "Temperature (C)", 
                            "Temperature setpoint (C)", "Temperature error (C)",
                            "Temperature PID output", "Temperature Kp",
                            "Temperature Ki", "Temperature Kd"])
            
            temp_samples = len([x for x in cls.temperature_readings if x != ""])
            if temp_samples > 0:
                time_interval = total_time / (temp_samples - 1) if temp_samples > 1 else 0
                
                for i in range(temp_samples):
                    current_time = i * time_interval
                    row = [f"{current_time:.3f}",
                          cls.temperature_readings[i] if i < len(cls.temperature_readings) else "",
                          cls.temperature_setpoint[i] if i < len(cls.temperature_setpoint) else "",
                          cls.temperature_error[i] if i < len(cls.temperature_error) else "",
                          cls.temperature_pid_output[i] if i < len(cls.temperature_pid_output) else "",
                          cls.temperature_kp[i] if i < len(cls.temperature_kp) else "",
                          cls.temperature_ki[i] if i < len(cls.temperature_ki) else "",
                          cls.temperature_kd[i] if i < len(cls.temperature_kd) else ""]
                    writer.writerow(row)
            
            # Separadores entre tablas
            writer.writerow([])
            writer.writerow([])
            
            # Diameter Table
            writer.writerow(["DIAMETER DATA"])
            writer.writerow(["Elapsed Time (s)", "Diameter (mm)",
                            "Diameter setpoint (mm)", "Fan duty cycle (%)"])
            
            diameter_samples = len([x for x in cls.diameter_readings if x != ""])
            if diameter_samples > 0:
                time_interval = total_time / (diameter_samples - 1) if diameter_samples > 1 else 0
                
                for i in range(diameter_samples):
                    current_time = i * time_interval
                    row = [f"{current_time:.3f}",
                          cls.diameter_readings[i] if i < len(cls.diameter_readings) else "",
                          cls.diameter_setpoint[i] if i < len(cls.diameter_setpoint) else "",
                          cls.fan_duty_cycle[i] if i < len(cls.fan_duty_cycle) else "0"]
                    writer.writerow(row)
            
            # Separadores entre tablas
            writer.writerow([])
            writer.writerow([])
            
            # Motor Table
            writer.writerow(["MOTOR DATA"])
            writer.writerow(["Elapsed Time (s)", "Extruder RPM",
                            "Spooler setpoint (RPM)", "Spooler RPM",
                            "Spooler gain", "Spooler oscilation period"])
            
            motor_samples = len([x for x in cls.spooler_rpm if x != ""])
            if motor_samples > 0:
                time_interval = total_time / (motor_samples - 1) if motor_samples > 1 else 0
                
                for i in range(motor_samples):
                    current_time = i * time_interval
                    row = [f"{current_time:.3f}",
                          cls.extruder_rpm[i] if i < len(cls.extruder_rpm) else "",
                          cls.spooler_setpoint[i] if i < len(cls.spooler_setpoint) else "",
                          cls.spooler_rpm[i] if i < len(cls.spooler_rpm) else "",
                          cls.spooler_gain[i] if i < len(cls.spooler_gain) else "",
                          cls.spooler_oscilation_period[i] if i < len(cls.spooler_oscilation_period) else ""]
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
