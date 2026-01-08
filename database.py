import yaml
import csv

class Database():
    """Class to store the raw data and generate the CSV file"""
    time_readings = []
    
    temperature_timestamps = []  # For future temperature measurements
    temperature_delta_time = []
    temperature_readings = []
    temperature_filtered = []
    temperature_setpoint = []
    temperature_dutycycle = []
    temperature_kp = []
    temperature_ki = []
    temperature_kd = []

    extruder_motor_timestamps = []
    extruder_motor_rpm = []
    
    camera_timestamps = []  # Timestamps for diameter measurements
    diameter_delta_time = []
    diameter_readings = []
    diameter_setpoint = []

    spooler_timestamps = []  # For future spooler measurements
    spooler_delta_time = []
    spooler_readings = []
    spooler_setpoint = []
    spooler_dutycycle = []
    spooler_kp = []
    spooler_ki = []
    spooler_kd = []

    fan_timestamps = []
    fan_delta_time = []
    fan_duty_cycle = []

    @classmethod
    def generate_csv(cls, filename: str) -> None:
        """Generate a CSV file with the data"""
        filename = filename + ".csv"
        with open(filename, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
        
            # Obtener el tiempo total de ejecución
            # total_time = cls.time_readings[-1] if cls.time_readings else 0
            
            # Temperature Table con timestamps reales
            writer.writerow(["TEMPERATURE DATA"])
            writer.writerow(["Timestamp (s)", 
                             "dt (s)", 
                             "Temperature (C)",
                             "Temperature filtered (C)", 
                             "Temperature setpoint (C)", 
                             "Temperature Duty Cycle %", 
                             "Temperature Kp",
                             "Temperature Ki", 
                             "Temperature Kd"])
            
            temp_samples = len([x for x in cls.temperature_readings if x != ""])
            for i in range(temp_samples):
                row = [f"{cls.temperature_timestamps[i]:.3f}" if i < len(cls.temperature_timestamps) else "",
                        f"{cls.temperature_delta_time[i]:.3f}" if i < len(cls.temperature_delta_time) else "",
                        cls.temperature_readings[i] if i < len(cls.temperature_readings) else "",
                        cls.temperature_filtered[i] if i < len(cls.temperature_filtered) else "",
                        cls.temperature_setpoint[i] if i < len(cls.temperature_setpoint) else "",
                        cls.temperature_dutycycle[i] if i < len(cls.temperature_dutycycle) else "",
                        cls.temperature_kp[i] if i < len(cls.temperature_kp) else "",
                        cls.temperature_ki[i] if i < len(cls.temperature_ki) else "",
                        cls.temperature_kd[i] if i < len(cls.temperature_kd) else ""]
                writer.writerow(row)

            
            
            # Separadores entre tablas
            writer.writerow([])
            writer.writerow([])
            
            # Extruder Motor Table with actual timestamps
            writer.writerow(["EXTRUDER MOTOR DATA"])
            writer.writerow(["Timestamp (s)", 
                             "Extruder RPM",])
        
            extruder_motor_samples = len(cls.extruder_motor_timestamps)
            for i in range(extruder_motor_samples):
                row = [f"{cls.extruder_motor_timestamps[i]:.3f}" if i < len(cls.extruder_motor_timestamps) else "",
                      cls.extruder_motor_rpm[i] if i < len(cls.extruder_motor_rpm) else ""]
                writer.writerow(row)

            


            # Separadores entre tablas
            writer.writerow([])
            writer.writerow([])
            
            # Cooling Table with actual timestamps
            writer.writerow(["COOLING DATA"])
            writer.writerow(["Timestamp (s)",
                             "dt (s)",  
                             "Fan Duty Cycle %",])
        
            fan_samples = len(cls.fan_timestamps)
            for i in range(fan_samples):
                row = [f"{cls.fan_timestamps[i]:.3f}" if i < len(cls.fan_timestamps) else "",
                        f"{cls.dan_delta_time[i]:.3f}" if i < len(cls.fan_delta_time) else "",
                        cls.fan_duty_cycle[i] if i < len(cls.fan_duty_cycle) else ""]
                writer.writerow(row)




            # Separadores entre tablas
            writer.writerow([])
            writer.writerow([])
            
            # Diameter Table with actual timestamps
            writer.writerow(["DIAMETER DATA"])
            writer.writerow(["Timestamp (s)", 
                             "dt (s)",
                             "Diameter (mm)",
                             "Diameter setpoint (mm)"])
        
            diameter_samples = len(cls.diameter_readings)
            for i in range(diameter_samples):
                row = [f"{cls.camera_timestamps[i]:.3f}" if i < len(cls.camera_timestamps) else "",
                       f"{cls.diameter_delta_time[i]:.3f}" if i < len(cls.diameter_delta_time) else "",
                      cls.diameter_readings[i] if i < len(cls.diameter_readings) else "",
                      cls.diameter_setpoint[i] if i < len(cls.diameter_setpoint) else ""]
                writer.writerow(row)
            



            # Separadores entre tablas
            writer.writerow([])
            writer.writerow([])
            
            # SPOOLER Motor Table
            writer.writerow(["SPOOLER DATA"])
            writer.writerow(["Timestamp (s)",
                             "dt (s)",  
                             "Spooler RPM",
                             "Spooler setpoint (RPM)", 
                             "Spooler Duty Cycle %", 
                             "Spooler Kp", 
                             "Spooler Ki", 
                             "Spooler Kd"])
            
            motor_samples = len([x for x in cls.spooler_rpm if x != ""])
            for i in range(motor_samples):
                row = [f"{cls.spooler_timestamps[i]:.3f}" if i < len(cls.spooler_timestamps) else "",
                       f"{cls.spooler_delta_time[i]:.3f}" if i < len(cls.spooler_delta_time) else "",
                      cls.spooler_readings[i] if i < len(cls.spooler_readings) else "",
                      cls.spooler_setpoint[i] if i < len(cls.spooler_setpoint) else "",
                      cls.spooler_dutycycle[i] if i < len(cls.spooler_dutycycle) else "",
                      cls.spooler_kp[i] if i < len(cls.spooler_kp) else "",  
                      cls.spooler_ki[i] if i < len(cls.spooler_ki) else "",  
                      cls.spooler_kd[i] if i < len(cls.spooler_kd) else ""]  
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

