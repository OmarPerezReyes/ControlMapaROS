#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#define TRIG_PIN 12
#define ECHO_PIN 13

ros::NodeHandle nh;

// Publishers
std_msgs::String control_msg;
ros::Publisher pub_control("control", &control_msg);

std_msgs::Float32 distance_msg;
ros::Publisher pub("ultrasonicos_pub", &distance_msg);

std_msgs::Float32 left_tra_msg;
std_msgs::Float32 center_tra_msg;
std_msgs::Float32 right_tra_msg;
ros::Publisher pub_left("left_tra_pub", &left_tra_msg);
ros::Publisher pub_center("center_tra_pub", &center_tra_msg);
ros::Publisher pub_right("right_tra_pub", &right_tra_msg);

std_msgs::String decision_msg;
ros::Publisher pub_decision("decisiones_pub", &decision_msg);

// Variables de estado y velocidad
volatile int Left_Tra_Value;
volatile int Center_Tra_Value;
volatile int Right_Tra_Value;
volatile int speed = 60;
bool automatic_mode = false;

void controlCallback(const std_msgs::String &msg) {
    String data = String(msg.data);
    if (data.indexOf("aut") != -1) {
        if (!automatic_mode) {
            automatic_mode = true;
            nh.loginfo("Modo Automático activado.");
        }
    } else {
        if (automatic_mode) {
            automatic_mode = false;
            nh.loginfo("Modo Manual activado.");
            STOP();
        }
    }
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
            Move_Backward(30);
        }
    }
}

// Subscribers
ros::Subscriber<std_msgs::String> sub_control("control", controlCallback);
ros::Subscriber<std_msgs::String> sub_decision("decisiones_pub", decisionCallback);

void setup() {
    Serial.begin(115200);
    nh.initNode();
    nh.advertise(pub);
    nh.advertise(pub_left);
    nh.advertise(pub_center);
    nh.advertise(pub_right);
    nh.advertise(pub_decision);
    nh.subscribe(sub_control);
    nh.subscribe(sub_decision);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(1, INPUT);
    pinMode(2, INPUT);
    pinMode(3, INPUT);
}

void Infrared_Tracing(int speed) {
    Left_Tra_Value = analogRead(3);
    Center_Tra_Value = analogRead(1);
    Right_Tra_Value = analogRead(2);
    if (automatic_mode) {
        if (Left_Tra_Value < 200 && Center_Tra_Value > 200 && Right_Tra_Value < 200) {
            Move_Forward(speed);
            decision_msg.data = "Recto";
        } else if (Left_Tra_Value > 200 && Center_Tra_Value < 200 && Right_Tra_Value > 200) {
            Move_Forward(speed);
            decision_msg.data = "Recto";
        } else if (Left_Tra_Value > 200 && Center_Tra_Value > 200 && Right_Tra_Value < 200) {
            vuelta_izquierda(speed, 2.3);
            decision_msg.data = "Izquierda";
        } else if (Left_Tra_Value < 200 && Center_Tra_Value > 200 && Right_Tra_Value > 200) {
            vuelta_derecha(speed, 2.3);
            decision_msg.data = "Derecha";
        } else if (Left_Tra_Value < 200 && Center_Tra_Value < 200 && Right_Tra_Value > 200) {
            vuelta_derecha(speed, 2.8);
            decision_msg.data = "Derecha";
        } else if (Left_Tra_Value > 200 && Center_Tra_Value < 200 && Right_Tra_Value < 200) {
            vuelta_izquierda(speed, 2.8);
            decision_msg.data = "Izquierda";
        } else if (Left_Tra_Value > 200 && Center_Tra_Value > 200 && Right_Tra_Value > 200) {
            STOP();
            decision_msg.data = "Detenerse";
        } else if (Left_Tra_Value < 200 && Center_Tra_Value < 200 && Right_Tra_Value < 200) {
            Move_Backward(30);
            decision_msg.data = "Reversa";
        }
    }
    pub_decision.publish(&decision_msg);
}

void STOP() {
    digitalWrite(2, LOW);
    analogWrite(5, 0);
    digitalWrite(4, HIGH);
    analogWrite(6, 0);
}

void Move_Forward(int car_speed) {
    digitalWrite(2, HIGH);
    analogWrite(5, car_speed);
    digitalWrite(4, LOW);
    analogWrite(6, car_speed);
}

void Move_Backward(int car_speed) {
    digitalWrite(2, LOW);
    analogWrite(5, car_speed);
    digitalWrite(4, HIGH);
    analogWrite(6, car_speed);
}

void vuelta_derecha(int car_speed, float k) {
    digitalWrite(2, HIGH);
    analogWrite(5, car_speed * k);
    digitalWrite(4, LOW);
    analogWrite(6, car_speed);
}

void vuelta_izquierda(int car_speed, float k) {
    digitalWrite(2, HIGH);
    analogWrite(5, car_speed);
    digitalWrite(4, LOW);
    analogWrite(6, car_speed * k);
}

float checkdistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    return pulseIn(ECHO_PIN, HIGH) / 58.00;
}

void conexion() {
    if (Serial.available() > 0) {
        int dato = Serial.parseInt();
        switch (dato) {
            case 1:
                Infrared_Tracing(speed);
                break;
            case 3:
                speed = 60;
                break;
            case 5:
                STOP();
                break;
            case 8:
                Move_Forward(speed);
                break;
            case 2:
                Move_Backward(30);
                break;
            case 6:
                vuelta_derecha(speed, 2);
                break;
            case 4:
                vuelta_izquierda(speed, 2);
                break;
        }
    }
}

void loop() {
    nh.spinOnce();

    if (automatic_mode) {
        float d = checkdistance();
        distance_msg.data = d;
        pub.publish(&distance_msg);

        left_tra_msg.data = Left_Tra_Value;
        center_tra_msg.data = Center_Tra_Value;
        right_tra_msg.data = Right_Tra_Value;

        pub_left.publish(&left_tra_msg);
        pub_center.publish(&center_tra_msg);
        pub_right.publish(&right_tra_msg);

        if (d < 8) {
            Move_Backward(40);
            decision_msg.data = "Reversa";
            pub_decision.publish(&decision_msg);
        } else if (d < 15) {
            STOP();
            decision_msg.data = "Detenerse";
            pub_decision.publish(&decision_msg);
        } else {
            conexion();
            Infrared_Tracing(speed);
        }
    }

    delay(100);
}
