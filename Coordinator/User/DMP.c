#include "DMP.h"
#include "bsp_usart1.h"

#define DEFAULT_MPU_HZ  (100)
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static void run_self_test(void)
{
    int result;
//    char test_packet[4] = {0};
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		Uart1_Put_String("setting bias succesfully ......\n");
    }
	else
	{
		Uart1_Put_String("bias has not been modified ......\n");
	}
}

void DMP_Init(void)
{
	int result;
	result = mpu_init();
 if(!result)
  {
	  Uart1_Put_String("mpu initialization complete......\n ");
	  //mpu_set_sensor
	  if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
	  {
	  	 Uart1_Put_String("mpu_set_sensor complete ......\n");
	  }
	  else
	  {
	  	 Uart1_Put_String("mpu_set_sensor come across error ......\n");
	  }
	  //mpu_configure_fifo
	  if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
	  {
	  	 Uart1_Put_String("mpu_configure_fifo complete ......\n");
	  }
	  else
	  {
	  	 Uart1_Put_String("mpu_configure_fifo come across error ......\n");
	  }
	  //mpu_set_sample_rate
	  if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
	  {
	  	 Uart1_Put_String("mpu_set_sample_rate complete ......\n");
	  }
	  else
	  {
	  	 Uart1_Put_String("mpu_set_sample_rate error ......\n");
	  }
	  //dmp_load_motion_driver_firmvare
	  if(!dmp_load_motion_driver_firmware())
	  {
	  	Uart1_Put_String("dmp_load_motion_driver_firmware complete ......\n");
	  }
	  else
	  {
	  	Uart1_Put_String("dmp_load_motion_driver_firmware come across error ......\n");
	  }
	  //dmp_set_orientation
	  if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
	  {
	  	 Uart1_Put_String("dmp_set_orientation complete ......\n");
	  }
	  else
	  {
	  	 Uart1_Put_String("dmp_set_orientation come across error ......\n");
	  }
	  //dmp_enable_feature
	  if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
	        DMP_FEATURE_GYRO_CAL))
	  {
	  	 Uart1_Put_String("dmp_enable_feature complete ......\n");
	  }
	  else
	  {
	  	 Uart1_Put_String("dmp_enable_feature come across error ......\n");
	  }
	  //dmp_set_fifo_rate
	  if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
	  {
	  	 Uart1_Put_String("dmp_set_fifo_rate complete ......\n");
	  }
	  else
	  {
	  	 Uart1_Put_String("dmp_set_fifo_rate come across error ......\n");
	  }
	  run_self_test();
	  if(!mpu_set_dmp_state(1))
	  {
	  	 Uart1_Put_String("mpu_set_dmp_state complete ......\n");
	  }
	  else
	  {
	  	 Uart1_Put_String("mpu_set_dmp_state come across error ......\n");
	  }
  }	
}