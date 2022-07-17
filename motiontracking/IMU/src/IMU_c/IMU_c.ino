
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// --- offset ---
double offset_setted = false;
double gx,gy,gz;
double a_offset_setted = false;

// --- Quaternions ---

double qa_imu[4]; //acceleration in IMU frame system
double q_delta_offset[4];
double qa_lab[4]; //acceleration in lab frame system
double q_omega[4]={1,0,0,0};
double q_delta[4];
double q_identity[4]={1,0,0,0};

void  QUAT_create_vec_quat(double a, double b, double c,double q[])
{
  q[0]=0;
  q[1]=a;
  q[2]=b;
  q[3]=c;
}

double QUAT_norm(double q[])
{
  return q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
  
}
void QUAT_print(double q[])
{
  Serial.print("q_w:");
  Serial.print(q[0]);
  Serial.print(" ");

  Serial.print("q_x:");
  Serial.print(q[1]);
  Serial.print(" ");

  Serial.print("q_y:");
  Serial.print(q[2]);
  Serial.print(" ");

  Serial.print("q_z:");
  Serial.print(q[3]);
  Serial.print(" -- norma: ");
  Serial.print(QUAT_norm(q));
  Serial.print(" -- theta: ");
  Serial.print(2*acos(q[0])*360/6.28);
  Serial.println("°");
}

void  QUAT_create_rot_quat(double dt_s,double omx, double omy, double omz,double q[])
{
 
  
  double omega=sqrt(omx*omx+omy*omy+omz*omz);
  if(omega==0)
  {
    q[0]=1;
    q[1]=0;
    q[2]=0;
    q[3]=0;
    //Serial.println("Gimbal Lock");
    return;
  }
  double theta=omega*dt_s/2.;
  double cs =cos(theta);
  double sn =sin(theta);
  q[0]=cs;
  q[1]=omx*sn/omega;
  q[2]=omy*sn/omega;
  q[3]=omz*sn/omega;
}

void QUAT_mul(double q[],double p[],double q_out[])
{
  q_out[0]=q[0]*p[0]-q[1]*p[1]-q[2]*p[2]-q[3]*p[3];
  q_out[1]=q[0]*p[1]+q[1]*p[0]+q[2]*p[3]-q[3]*p[2];
  q_out[2]=q[0]*p[2]-q[1]*p[3]+q[2]*p[0]+q[3]*p[1];
  q_out[3]=q[0]*p[3]+q[1]*p[2]-q[2]*p[1]+q[3]*p[0];
}

void QUAT_inv(double q[],double q_out[])
{
  double n2=q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
  q_out[0]=q[0]/n2;
  q_out[1]=-q[1]/n2;
  q_out[2]=-q[2]/n2;
  q_out[3]=-q[3]/n2;
}

// --- Time ---
unsigned long last_time = millis();
unsigned long last_measurement_time = millis();


void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  int millisecs = 10;
  double dt=(double)millisecs/1000.;
  //dt=(millis()-last_measurement_time)*1000.;
  dt=0.01;

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  last_measurement_time=millis();

  //Create IMU quaternion
  QUAT_create_vec_quat(a.acceleration.x, a.acceleration.y, a.acceleration.z,qa_imu);
  
  //Create IMU rotation quaternion
  if(!offset_setted)
  {
    gx = g.gyro.x;
    gy = g.gyro.y;
    gz = g.gyro.z;
    offset_setted = true;
  }
  QUAT_create_rot_quat(dt,g.gyro.x-gx, g.gyro.y-gy, g.gyro.z-gz,q_delta);
  //QUAT_create_rot_quat(dt,0.0, 0, 0.0,q_delta);
  
  //Update q_omega
  double q_tmp[4];
  QUAT_mul(q_omega,q_delta,q_tmp);
  
  q_omega[0]=q_tmp[0];  
  q_omega[1]=q_tmp[1];
  q_omega[2]=q_tmp[2];
  q_omega[3]=q_tmp[3];

  //Calculate
  double q_inv[4];
  QUAT_inv(q_omega,q_inv);
  QUAT_mul(q_omega,qa_imu,q_tmp);
  QUAT_mul(q_tmp,q_inv,qa_lab);
  
  
  if(millis()-last_time>2500)
  {
    last_time=millis();
  /* Print out the values */
   Serial.println("--vectors--");
   Serial.print("Acceleration X: ");
   Serial.print(a.acceleration.x);
   Serial.print("/");
   Serial.print(qa_lab[1]);
   Serial.print(", Y: ");
   Serial.print(a.acceleration.y);
   Serial.print("/");
   Serial.print(qa_lab[2]);
   Serial.print(", Z: ");
   Serial.print(a.acceleration.z);
   Serial.print("/");
    Serial.print(qa_lab[3]);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");
  
    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");

    Serial.println("--quaternions--");
    Serial.println("q_delta");
    QUAT_print(q_delta);
    Serial.println("q_omega");
    QUAT_print(q_omega);
   
    
  }  
  

  
  delay(millisecs);
}
