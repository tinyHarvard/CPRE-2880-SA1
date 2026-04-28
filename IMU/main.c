#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "uart.h"
#include "Timer.h"
#include "bno055.h"

int main(void)
{
    char buf[256];

    uart_init();
    timer_init();
    bno_init();
    timer_waitMillis(700);

    bno_t *bno = bno_alloc();

    uart_sendStr("\r\nBNO055 IMU ready\r\n");
    uart_sendStr("--------------------------------------------------\r\n");

    while (1)
    {
        bno_update(bno);

        // Accel (raw units: 1 m/s^2 = 100 LSB)
        sprintf(buf,
                "Accel    X=%6d Y=%6d Z=%6d\r\n",
                bno->accel.x, bno->accel.y, bno->accel.z);
        uart_sendStr(buf);

        // Magnetometer (1 uT = 16 LSB)
        sprintf(buf,
                "Mag      X=%6d Y=%6d Z=%6d\r\n",
                bno->mag.x, bno->mag.y, bno->mag.z);
        uart_sendStr(buf);

        // Gyroscope (1 dps = 16 LSB)
        sprintf(buf,
                "Gyro     X=%6d Y=%6d Z=%6d\r\n",
                bno->gyro.x, bno->gyro.y, bno->gyro.z);
        uart_sendStr(buf);

        // Euler angles (1 deg = 16 LSB)
        sprintf(buf,
                "Euler    H=%6d R=%6d P=%6d\r\n",
                bno->euler.heading, bno->euler.roll, bno->euler.pitch);
        uart_sendStr(buf);

        // Quaternion (1 quat = 2^14 LSB)
        sprintf(buf,
                "Quat     W=%6d X=%6d Y=%6d Z=%6d\r\n",
                bno->quat.w, bno->quat.x, bno->quat.y, bno->quat.z);
        uart_sendStr(buf);

        // Linear acceleration (gravity removed; 1 m/s^2 = 100 LSB)
        sprintf(buf,
                "LinAcc   X=%6d Y=%6d Z=%6d\r\n",
                bno->linacl.x, bno->linacl.y, bno->linacl.z);
        uart_sendStr(buf);

        // Gravity vector (1 m/s^2 = 100 LSB)
        sprintf(buf,
                "Gravity  X=%6d Y=%6d Z=%6d\r\n",
                bno->grav.x, bno->grav.y, bno->grav.z);
        uart_sendStr(buf);

        // Temperature (deg C, 1 LSB = 1 deg)
        sprintf(buf, "Temp     %d C\r\n", bno->temp);
        uart_sendStr(buf);

        uart_sendStr("--------------------------------------------------\r\n");

        timer_waitMillis(250);
    }
}
