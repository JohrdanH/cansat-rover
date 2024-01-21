# Cansat Rover

Welcome to the "Cansat Rover" GitHub repository! This project focuses on the development of a Cansat Rover controlled by an Arduino Uno. The Rover is equipped with two DC motors for movement, an Inertial Motion Unit MPU9250 with an integrated GPS sensor, and a compass sensor QMC5883L.

## Contents

1. [Read_GPS_MPU](#read_gps_mpu)
2. [test_motors_hmc5883L](#test_motors_hmc5883l)

## Read_GPS_MPU

The `Read_GPS_MPU` folder contains the Arduino code to read GPS readings from the MPU9250 Inertial Motion Unit. This code interfaces with the GPS sensor to obtain location information from the Rover.

### Instructions

- Explore the Arduino code in the `Read_GPS_MPU` directory.
- Connect your Arduino Uno to the MPU9250 and upload the code to read and display GPS readings.

## test_motors_hmc5883L

The `test_motors_hmc5883L` folder contains the Arduino code to control the Cansat Rover's movement using two DC motors while calculating its position based on GPS readings from the MPU9250 and compass data from the QMC5883L sensor.

### Instructions

- Review the Arduino code in the `test_motors_hmc5883L` directory.
- Connect your Arduino Uno to the DC motors, MPU9250, and QMC5883L.
- Upload the code to the Arduino Uno to control the Rover's movement and obtain position information.

## Contributing

If you find any issues, have suggestions for improvements, or want to contribute additional features, feel free to contribute! Fork the repository, make your changes, and submit a pull request.

## License

This project is licensed under the [MIT License](LICENSE), allowing for open collaboration and sharing.

Feel free to explore the project, experiment with the code, and adapt it to your needs. Happy Rover building! 🚀
