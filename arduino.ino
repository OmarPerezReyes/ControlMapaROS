#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#define TRIG_PIN 12
#define ECHO_PIN 13

// Definición de pines
const int pinD2 = 2;
const int pinD4 = 4;
const int pinPWM1 = 5;  // Pin PWM para el motor 1
const int pinPWM2 = 6;  // Pin PWM para el motor 2

ros::NodeHandle nh;

// Publishers
std_msgs::Float32 distance_msg;
ros::Publisher pub_distance("ultrasonicos_pub", &distance_msg);

std_msgs::Float32 left_tra_msg;
std_msgs::Float32 center_tra_msg;
std_msgs::Float32 right_tra_msg;
ros::Publisher pub_left("left_tra_pub", &left_tra_msg);
ros::Publisher pub_center("center_tra_pub", &center_tra_msg);
ros::Publisher pub_right("right_tra_pub", &right_tra_msg);

std_msgs::String decision_msg;
ros::Publisher pub_decision("decisiones_pub", &decision_msg);

std_msgs::String qr_msg;
ros::Publisher pub_qr("deteccionQr", &qr_msg);

// Variables de estado y velocidad
volatile int Left_Tra_Value = 0;
volatile int Center_Tra_Value = 0;
volatile int Right_Tra_Value = 0;
volatile int speed = 10;
bool automatic_mode = false;

// Variables para controlar la frecuencia de las lecturas de los sensores
unsigned long last_sensor_reading = 0;
unsigned long sensor_interval = 100;  // Intervalo de 100ms para leer sensores

unsigned long last_pub_time = 0;
unsigned long pub_interval = 1000; // Intervalo de publicación de 1 segundo

void controlCallback(const std_msgs::String &msg) {
    String data = String(msg.data);
    if (data == "aut" && !automatic_mode) {
        automatic_mode = true;
        nh.loginfo("Modo Automático activado.");
    } else if (data != "aut" && automatic_mode) {
        automatic_mode = false;
        nh.loginfo("Modo Manual activado.");
        STOP();
    }
}

void qrCallback(const std_msgs::String &msg) {
    //nh.loginfo(msg.data);
    String data = String(msg.data);
    if (data == "Alto") {
        nh.loginfo("Se detectó Alto");
        STOP();
        delay(4000);
    }
    /* else {
        nh.loginfo("Se detectó Avanzar");
        Move_Forward(speed);
    }*/
}


void decisionCallback(const std_msgs::String &msg) {
    if (!automatic_mode) {
        String decision = String(msg.data);
        if (decision == "Detenerse") {
            STOP();
        } else if (decision == "Izquierda") {
            vuelta_izquierda(speed, 2.3);
        } else if (decision == "Derecha") {
            vuelta_derecha(speed, 2.3);
        } else if (decision == "Recto") {
            Move_Forward(speed);
        } else if (decision == "Reversa") {
            Move_Backward(15);
        }
        else{
          STOP();
        }
    }
}

// Subscribers
ros::Subscriber<std_msgs::String> sub_control("control", controlCallback);
ros::Subscriber<std_msgs::String> sub_decision("decisiones_pub", decisionCallback);
ros::Subscriber<std_msgs::String> sub_qr("deteccionQr", qrCallback);

void setup() {
    Serial.begin(57600);
    nh.initNode();

    // Publicadores y suscriptores
    nh.advertise(pub_distance);
    nh.advertise(pub_left);
    nh.advertise(pub_center);
    nh.advertise(pub_right);
    nh.advertise(pub_qr); // Añadido para deteccionQr
    nh.advertise(pub_decision);
    nh.subscribe(sub_control);
    nh.subscribe(sub_qr);
    nh.subscribe(sub_decision);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(pinD2, OUTPUT);
    pinMode(pinD4, OUTPUT);
    pinMode(pinPWM1, OUTPUT);
    pinMode(pinPWM2, OUTPUT);


    // Publicación inicial para deteccionQr
    qr_msg.data = "8"; // Publica el valor "8" al tópico deteccionQr
    pub_qr.publish(&qr_msg);
    
    // Mensaje de inicialización
    nh.loginfo("Nodo de ROS inicializado.");
}

void Infrared_Tracing(int speed) {
    speed = 10;
    if (Left_Tra_Value < 200 && Center_Tra_Value > 200 && Right_Tra_Value < 200) {
      
        Move_Forward(speed);
        decision_msg.data = "Recto";
    } else if (Left_Tra_Value > 200 && Center_Tra_Value < 200 && Right_Tra_Value > 200) {
        Move_Forward(speed);
        decision_msg.data = "Recto";
    } else if (Left_Tra_Value > 200 && Center_Tra_Value > 200 && Right_Tra_Value < 200) {
        vuelta_izquierda(speed, .8);
        decision_msg.data = "Izquierda";
    } else if (Left_Tra_Value < 200 && Center_Tra_Value > 200 && Right_Tra_Value > 200) {
        vuelta_derecha(speed, .8);
        decision_msg.data = "Derecha";
    } else if (Left_Tra_Value < 200 && Center_Tra_Value < 200 && Right_Tra_Value > 200) {
        vuelta_derecha(speed, 1.3);
        decision_msg.data = "Derecha";
    } else if (Left_Tra_Value > 200 && Center_Tra_Value < 200 && Right_Tra_Value < 200) {
        vuelta_izquierda(speed, 1.3);
        decision_msg.data = "Izquierda";
    } else if (Left_Tra_Value > 200 && Center_Tra_Value > 200 && Right_Tra_Value > 200) {
        STOP();
        decision_msg.data = "Detenerse";
    } else if (Left_Tra_Value < 200 && Center_Tra_Value < 200 && Right_Tra_Value < 200) {
        Move_Backward(5);
        decision_msg.data = "Reversa";
    }

    pub_decision.publish(&decision_msg);
}

void STOP() {
    analogWrite(pinPWM1, 0);  // Detiene motor 1
    analogWrite(pinPWM2, 0);  // Detiene motor 2
    digitalWrite(pinD2, LOW); // Apaga motor 1
    digitalWrite(pinD4, LOW); // Apaga motor 2
}

void Move_Backward(int car_speed) {
    digitalWrite(pinD2, LOW);
    analogWrite(pinPWM1, 38); // Motor 1 en retroceso
    digitalWrite(pinD4, HIGH);
    analogWrite(pinPWM2, 38); // Motor 2 en retroceso
}

void Move_Forward(int car_speed) {
    digitalWrite(pinD2, HIGH);  // Motor 1 adelante
    analogWrite(pinPWM1, 45); // Controla la velocidad del motor 1
    digitalWrite(pinD4, LOW);   // Motor 2 en dirección adelante
    analogWrite(pinPWM2, 45); // Controla la velocidad del motor 2
}

void vuelta_derecha(int car_speed, float k) {
    digitalWrite(pinD2, HIGH);  // Motor 1 adelante
    analogWrite(pinPWM1, 45);
    digitalWrite(pinD4, HIGH);  // Motor 2 adelante
    analogWrite(pinPWM2, 45);
}

void vuelta_izquierda(int car_speed, float k) {
    digitalWrite(pinD2, LOW);  // Motor 1 retroceso
    analogWrite(pinPWM1, 45);
    digitalWrite(pinD4, LOW);  // Motor 2 retroceso
    analogWrite(pinPWM2, 45);
}

float checkdistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    return pulseIn(ECHO_PIN, HIGH) / 58.00;
}

void loop() {
    nh.spinOnce(); // Procesa la comunicación con ROS, ejecuta callbacks y loginfo

    unsigned long currentMillis = millis();

    // Lectura de sensores cada 100 ms
    if (currentMillis - last_sensor_reading >= sensor_interval) {
        Left_Tra_Value = analogRead(3);
        Center_Tra_Value = analogRead(1);
        Right_Tra_Value = analogRead(2);
        last_sensor_reading = currentMillis;
    }

    // Publicación de datos cada 1 segundo
    if (currentMillis - last_pub_time >= pub_interval) {
        distance_msg.data = checkdistance();
        pub_distance.publish(&distance_msg);

        left_tra_msg.data = Left_Tra_Value;
        center_tra_msg.data = Center_Tra_Value;
        right_tra_msg.data = Right_Tra_Value;

        pub_left.publish(&left_tra_msg);
        pub_center.publish(&center_tra_msg);
        pub_right.publish(&right_tra_msg);

        last_pub_time = currentMillis;
    }

    // Modo automático
    if (automatic_mode) {
        speed = 10;
        float d = checkdistance();
        Infrared_Tracing(speed);
    }delay(10);}
