Sensors
######

Overview
********

| This sample is to read the data from the BME680 and LIS3MDL. The nRF52833 reads the data through I2C0 interface. You can see the devicetree in boards/arm/tautek/tautek.dts
| 
| &i2c0 {                         
| 	compatible = "nordic,nrf-twi";
| 	status = "okay";
| 	pinctrl-0 = <&i2c0_default>;
| 	pinctrl-1 = <&i2c0_sleep>;
| 	pinctrl-names = "default", "sleep";
| 
| 	bme680@76 {
| 		compatible = "bosch,bme680";
| 		reg = <0x76>;
| 	};
| 	lis3mdl@1c {
| 		compatible = "st,lis3mdl-magn";
| 		reg = <0x1c>;
| 	};
| 
| When you build the project, you can select the prj_logs.conf to print the log on the RTT Viewer. To print the log on the serial interface(Tx, Rx), you can use prj.conf.