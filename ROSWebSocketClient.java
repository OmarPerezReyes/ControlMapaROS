package com.example.controlmapavehiculo;

import android.app.Activity;
import android.os.Handler;
import android.util.Log;
import android.view.View;
import android.widget.ImageView;
import android.widget.TextView;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;
import org.json.JSONException;
import org.json.JSONObject;

import java.net.URI;
import java.net.URISyntaxException;

public class ROSWebSocketClient extends WebSocketClient {
    private final Activity activity;
    private final TextView txtDistance;
    private final TextView txtCenter;
    private final TextView txtRight;
    private final TextView txtLeft;
    private final TextView txtDecision;
    private final TextView tvSignalStatus;  // El TextView para "No señalización"
    private final ImageView imgSignal;  // El ImageView para mostrar la señalización


    // Constructor que recibe el URI del servidor y la Activity
    public ROSWebSocketClient(
            String serverUri, Activity activity,
            TextView txtDistance, TextView txtCenter,
            TextView txtRight, TextView txtLeft, TextView txtDecision, TextView tvSignalStatus, ImageView imgSignal)
            throws URISyntaxException {
        super(new URI(serverUri));  // Llama al constructor de WebSocketClient con la URI del servidor
        this.activity = activity;  // Asigna la actividad para actualizar la interfaz de usuario
        this.txtDistance = txtDistance;
        this.txtCenter = txtCenter;
        this.txtRight = txtRight;
        this.txtLeft = txtLeft;
        this.txtDecision = txtDecision;
        this.tvSignalStatus = tvSignalStatus;  // Asigna el TextView
        this.imgSignal = imgSignal;  // Asigna el ImageView
    }

    @Override
    public void onOpen(ServerHandshake handshake) {
        Log.d("ROSWebSocket", "Conexión establecida");  // Log para indicar que se ha establecido la conexión

        // Mensaje de suscripción para cada tópico de interés
        String subscribeUltrasonicos = "{ \"op\": \"subscribe\", \"topic\": \"/ultrasonicos_pub\" }";
        String subscribeCenter = "{ \"op\": \"subscribe\", \"topic\": \"/center_tra_pub\" }";
        String subscribeRight = "{ \"op\": \"subscribe\", \"topic\": \"/right_tra_pub\" }";
        String subscribeLeft = "{ \"op\": \"subscribe\", \"topic\": \"/left_tra_pub\" }";
        String subscribeDecision = "{ \"op\": \"subscribe\", \"topic\": \"/decisiones_pub\" }";
        String subscribeDetectionQr = "{ \"op\": \"subscribe\", \"topic\": \"/deteccionQr\" }";

        // Enviar los mensajes de suscripción al servidor
        send(subscribeUltrasonicos);
        send(subscribeCenter);
        send(subscribeRight);
        send(subscribeLeft);
        send(subscribeDecision);
        send(subscribeDetectionQr);  // Suscribe al tópico de detección de QR

    }

    @Override
    public void onMessage(final String message) {
        // Aquí se procesa el mensaje recibido según su tópico de origen.
        // Se asume que el mensaje tiene un campo "topic" para identificar su origen.
        activity.runOnUiThread(() -> {  // Actualiza la UI en el hilo de la interfaz
            try {
                // Procesa el mensaje como JSON
                org.json.JSONObject json = new org.json.JSONObject(message);
                String topic = json.getString("topic");  // Obtiene el nombre del tópico
                String data = json.getString("msg");  // Obtiene el mensaje

                // Actualiza la UI según el tópico del mensaje recibido
                switch (topic) {
                    case "/ultrasonicos_pub":
                        try {
                            JSONObject jsonObject = new JSONObject(data);  // Convierte "data" en JSON
                            double distance = jsonObject.getDouble("data"); // Extrae el número de distancia
                            txtDistance.setText(String.format("Distancia: %.8f cm", distance));  // Muestra la distancia
                        } catch (JSONException e) {
                            e.printStackTrace();  // Manejador de excepción si el JSON es inválido
                        }
                        break;

                    case "/center_tra_pub":
                        try {
                            JSONObject jsonObject = new JSONObject(data);  // Convierte "data" en JSON
                            double centerValue = jsonObject.getDouble("data"); // Extrae el valor del sensor central
                            txtCenter.setText(String.format("Sensor Centro: %.8f", centerValue));  // Muestra el valor
                        } catch (JSONException e) {
                            e.printStackTrace();  // Manejador de excepción
                        }
                        break;

                    case "/right_tra_pub":
                        try {
                            JSONObject jsonObject = new JSONObject(data);  // Convierte "data" en JSON
                            double rightValue = jsonObject.getDouble("data"); // Extrae el valor del sensor derecho
                            txtRight.setText(String.format("Sensor Derecha: %.8f", rightValue));  // Muestra el valor
                        } catch (JSONException e) {
                            e.printStackTrace();  // Manejador de excepción
                        }
                        break;

                    case "/left_tra_pub":
                        try {
                            JSONObject jsonObject = new JSONObject(data);  // Convierte "data" en JSON
                            double leftValue = jsonObject.getDouble("data"); // Extrae el valor del sensor izquierdo
                            txtLeft.setText(String.format("Sensor Izquierda: %.8f", leftValue));  // Muestra el valor
                        } catch (JSONException e) {
                            e.printStackTrace();  // Manejador de excepción
                        }
                        break;

                    case "/decisiones_pub":
                        try {
                            JSONObject jsonObject = new JSONObject(data);  // Convierte "data" en JSON
                            String decisionText = jsonObject.getString("data"); // Extrae el texto de decisión
                            txtDecision.setText("Decisión: " + decisionText); // Muestra la decisión
                        } catch (JSONException e) {
                            e.printStackTrace();  // Manejador de excepción
                        }
                        break;
                    case "/deteccionQr":
                        try {
                            JSONObject jsonObject = new JSONObject(data);  // Convierte "data" en JSON
                            String detectionMessage = jsonObject.getString("data"); // Extrae el mensaje de detección
                            tvSignalStatus.setVisibility(View.VISIBLE); // Muestra el texto
                            imgSignal.setVisibility(View.GONE); // Oculta la imagen
                            tvSignalStatus.setText("No señalización"); // Actualiza el texto
                            // Si el mensaje es "Alto", muestra la imagen
                            if ("Alto".equals(detectionMessage)) {
                                // Mostrar la imagen de señalización
                                imgSignal.setVisibility(View.VISIBLE);
                                tvSignalStatus.setVisibility(View.GONE); // Oculta el texto

                                // Crear un Handler para ejecutar una acción después de 3 segundos (3000 ms)
                                new Handler().postDelayed(new Runnable() {
                                    @Override
                                    public void run() {
                                        // Después de 3 segundos, mostrar "No señalización"
                                        tvSignalStatus.setVisibility(View.VISIBLE); // Muestra el texto
                                        imgSignal.setVisibility(View.GONE); // Oculta la imagen
                                        tvSignalStatus.setText("No señalización"); // Actualiza el texto
                                    }
                                }, 3000); // 3000 milisegundos = 3 segundos
                            } else {
                                // Si no es "Alto", mostrar el texto "No señalización"
                                tvSignalStatus.setVisibility(View.VISIBLE); // Muestra el texto
                                imgSignal.setVisibility(View.GONE); // Oculta la imagen
                                tvSignalStatus.setText("No señalización"); // Actualiza el texto
                            }
                        } catch (JSONException e) {
                            e.printStackTrace(); // Manejo de errores si el JSON no es válido
                        }

                        break;

                }
            } catch (Exception e) {
                Log.e("ROSWebSocket", "Error al procesar mensaje: " + e.getMessage());  // Log de error en el procesamiento
            }
        });
    }

    @Override
    public void onClose(int code, String reason, boolean remote) {
        Log.d("ROSWebSocket", "Conexión cerrada: " + reason);  // Log cuando se cierra la conexión
    }

    @Override
    public void onError(Exception ex) {
        Log.e("ROSWebSocket", "Error: " + ex.getMessage());  // Log para errores de conexión o comunicación
    }
}
