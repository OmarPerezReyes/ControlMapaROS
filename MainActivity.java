package com.example.controlmapavehiculo;

import android.os.Bundle;
import androidx.appcompat.app.AppCompatActivity;
import java.net.URISyntaxException;

import android.util.Log;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Switch;
import android.text.Editable;
import android.text.TextWatcher;
import android.os.Handler;
import android.widget.ImageView;

public class MainActivity extends AppCompatActivity {
    private ROSWebSocketClient webSocketClient;  // Cliente WebSocket para conexión ROS
    private CanvasView canvasView;  // Vista personalizada para dibujar la trayectoria
    private Button btnForward, btnReverse, btnLeft1, btnRight1, btnStop;  // Botones de dirección
    private TextView txtDecision, tvSignalStatus;  // Texto que muestra decisiones actuales
    private Switch switchMode;  // Switch para alternar entre modo manual y automático
    boolean flag;  // Bandera para el estado del modo
    private Handler handler = new Handler();  // Handler para manejo de acciones programadas
    private Runnable sendStateRunnable;  // Runnable para enviar estado
    private Button btnResetCanvas;  // Botón para resetear el lienzo
    private ImageView imgSignal;  // ImageView para mostrar la señalización

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Asigna referencias a los elementos de la vista
        txtDecision = findViewById(R.id.tvDecision);
        canvasView = findViewById(R.id.canvasView);
        switchMode = findViewById(R.id.switchMode);
        tvSignalStatus = findViewById(R.id.tvSignalStatus);  // Referencia al TextView de "No señalización"
        imgSignal = findViewById(R.id.imgSignal);  // Referencia al ImageView de señalización


        // Configurar los botones de dirección
        btnForward = findViewById(R.id.btnForward);
        btnReverse = findViewById(R.id.btnReverse);
        btnLeft1 = findViewById(R.id.btnLeft);
        btnRight1 = findViewById(R.id.btnRight);
        btnStop = findViewById(R.id.btnStop);
        btnResetCanvas = findViewById(R.id.btnResetCanvas);

        // Inicializar la conexión WebSocket
        try {
            webSocketClient = new ROSWebSocketClient(
                    "ws://192.168.100.13:9090", //IP raspberry (ifconfig)
                    this,
                    findViewById(R.id.tvDistance),
                    findViewById(R.id.tvUltrasonic1),
                    findViewById(R.id.tvUltrasonic2),
                    findViewById(R.id.tvUltrasonic3),
                    findViewById(R.id.tvDecision),
                    tvSignalStatus,  // Pasar el TextView para "No señalización"
                    imgSignal  // Pasar el ImageVie
            );
            webSocketClient.connect();

            // Configura un TextWatcher para detectar cambios en txtDecision
            txtDecision.addTextChangedListener(new TextWatcher() {
                @Override
                public void beforeTextChanged(CharSequence s, int start, int count, int after) {}

                @Override
                public void onTextChanged(CharSequence s, int start, int before, int count) {
                    String currentText = s.toString();
                    if (flag) {  // Solo actualizar si flag está activado
                        updateCanvasDirection(currentText);
                    }
                }

                @Override
                public void afterTextChanged(Editable s) {}

            });
            // Listener para el botón de resetear el lienzo
            btnResetCanvas.setOnClickListener(v -> canvasView.resetCanvas());

            // Configurar los clics de los botones de dirección
            btnForward.setOnClickListener(v -> {
                canvasView.setDirection("up");
                sendDecisionMessage("Recto");
            });

            btnReverse.setOnClickListener(v -> {
                canvasView.setDirection("down");
                sendDecisionMessage("Reversa");
            });

            btnLeft1.setOnClickListener(v -> {
                canvasView.setDirection("left");
                sendDecisionMessage("Izquierda");
            });

            btnRight1.setOnClickListener(v -> {
                canvasView.setDirection("right");
                sendDecisionMessage("Derecha");
            });

            btnStop.setOnClickListener(v -> {
                canvasView.stopDrawing();
                sendDecisionMessage("Detenerse");
            });

            // Configuración del switch entre modo automático y manual
            switchMode.setOnCheckedChangeListener((buttonView, isChecked) -> {
                setButtonsEnabled(!isChecked);  // Desactivar botones en modo automático
                flag = isChecked;

                // Detener el dibujo al cambiar de estado
                canvasView.stopDrawing();
                String estado = flag ? "aut" : "man";
                String message = "{ \"op\": \"publish\", \"topic\": \"/control\", \"msg\": { \"data\": \"" + estado + "\" } }";
                Log.d("WebSocket", "Enviando mensaje: " + message);
                webSocketClient.send(message);  // Enviar el estado de modo por WebSocket
            });

        } catch (URISyntaxException e) {
            e.printStackTrace();
        }
    }

    // Enviar mensaje de decisión actual por WebSocket
    private void sendDecisionMessage(String decision) {
        String message = "{ \"op\": \"publish\", \"topic\": \"/decisiones_pub\", \"msg\": { \"data\": \"" + decision + "\" } }";
        Log.d("WebSocket", "Enviando mensaje: " + message);
        webSocketClient.send(message);
    }

    // Reinicia el runnable que envía el estado actual
    private void restartSendStateRunnable() {
        handler.removeCallbacks(sendStateRunnable);
        handler.post(sendStateRunnable);
    }

    // Habilitar o deshabilitar botones
    private void setButtonsEnabled(boolean enabled) {
        btnForward.setEnabled(enabled);
        btnReverse.setEnabled(enabled);
        btnLeft1.setEnabled(enabled);
        btnRight1.setEnabled(enabled);
        btnStop.setEnabled(enabled);
    }

    // Actualiza la dirección en el lienzo basada en el texto
    private void updateCanvasDirection(String text) {
        if (text.equals("Decisión: Recto")) {
            canvasView.setDirection("up");
        } else if (text.equals("Decisión: Derecha")) {
            canvasView.setDirection("right");
        } else if (text.equals("Decisión: Izquierda")) {
            canvasView.setDirection("left");
        } else if (text.equals("Decisión: Reversa")) {
            canvasView.setDirection("down");
        } else {
            canvasView.stopDrawing();
        }
    }

    // Al destruir la actividad, elimina los callbacks y cierra la conexión WebSocket
    @Override
    protected void onDestroy() {
        super.onDestroy();
        handler.removeCallbacks(sendStateRunnable);  // Detiene el runnable al destruir
        if (webSocketClient != null) {
            webSocketClient.close();
        }
    }
}
