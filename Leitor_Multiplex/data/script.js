let socket = new WebSocket(`ws://${location.hostname}:8080`);

const labels = [];
const pvData = [];
const mvData = [];

const ctx = document.getElementById('pidChart').getContext('2d');
const pidChart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: labels,
        datasets: [
            {
                label: 'Tensão Medida (PV)',
                borderColor: 'blue',
                data: pvData,
                fill: false,
                tension: 0.1
            },
            {
                label: 'Tensão Aplicada (MV)',
                borderColor: 'red',
                data: mvData,
                fill: false,
                tension: 0.1
            }
        ]
    },
    options: {
        responsive: true,
        animation: false,
        scales: {
            x: {
                title: { display: true, text: 'Tempo (s)' }
            },
            y: {
                title: { display: true, text: 'Tensão (V)' },
                suggestedMin: 0,
                suggestedMax: 3.5
            }
        }
    }
});

socket.onopen = () => {
    document.getElementById("status").textContent = "🟢 Conectado ao ESP32";
};

socket.onclose = () => {
    document.getElementById("status").textContent = "🔴 Desconectado";
};

socket.onerror = () => {
    document.getElementById("status").textContent = "❌ Erro na conexão";
};

socket.onmessage = (event) => {
    try {
        const data = JSON.parse(event.data);
        const tempo = data.time;
        const pv = parseFloat(data.PV);
        const mv = parseFloat(data.MV);

        labels.push(tempo);
        pvData.push(pv);
        mvData.push(mv);

        if (labels.length > 50) {
            labels.shift();
            pvData.shift();
            mvData.shift();
        }

        pidChart.update();
    } catch (e) {
        console.error("Erro ao processar mensagem:", e);
    }
};

function send(msg) {
    if (socket.readyState === WebSocket.OPEN) {
        socket.send(msg);
    } else {
        alert("Conexão WebSocket não está aberta.");
    }
}

function sendParams() {
    const sp = document.getElementById("sp").value;
    const kp = document.getElementById("kp").value;
    const ki = document.getElementById("ki").value;
    const kd = document.getElementById("kd").value;
    const planta = document.getElementById("planta").value;

    send(`setSp:${sp}`);
    send(`setKp:${kp}`);
    send(`setKi:${ki}`);
    send(`setKd:${kd}`);
    send(`setPlanta:${planta}`);
}

function sendStart() {
    send("start");
}

function sendStop() {
    send("stop");
}

const plantaGains = {
    1: { kp: 1.922, ki: 0.602, kd: 0.0 },
    2: { kp: 2.033, ki: 0.525, kd: 0.0 },
    3: { kp: 2.054, ki: 0.299, kd: 0.0 },
    4: { kp: 1.961, ki: 0.241, kd: 0.0 },
    5: null,
    6: null,
};

function autoFillGains() {
    const planta = parseInt(document.getElementById("planta").value);
    const gains = plantaGains[planta];
    if (gains) {
        document.getElementById("kp").value = gains.kp;
        document.getElementById("ki").value = gains.ki;
        document.getElementById("kd").value = gains.kd;
    } else {
        document.getElementById("kp").value = "";
        document.getElementById("ki").value = "";
        document.getElementById("kd").value = "";
    }
}
