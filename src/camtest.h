const char index_html[] PROGMEM = R"raw(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>Live Stream</title>
  <style>
    body {
      margin: 0;
      background-color: #000;
      display: flex;
      justify-content: center;
      align-items: center;
      height: 100vh;
    }
    canvas {
      max-width: 100%;
    }
  </style>
</head>
<body>
  <canvas id="canvas" width="640" height="480"></canvas>
  <script>
    async function getFrame() {
      try {
        let res = await fetch(window.location.origin + '/getFrame');
        let arr = await res.arrayBuffer();
        let data = new Uint8Array(arr);
        let canvas = document.getElementById("canvas");
        let ctx = canvas.getContext("2d");
        let imageData = ctx.createImageData(canvas.width, canvas.height);
        data.forEach((x, i) => {
          imageData.data[i * 4 + 0] = x;
          imageData.data[i * 4 + 1] = x;
          imageData.data[i * 4 + 2] = x;
          imageData.data[i * 4 + 3] = 255;
        });
        ctx.putImageData(imageData, 0, 0);
      } catch (e) {
        console.error(e);
      }
      setTimeout(getFrame, 30); // обновление ~33 кадра в секунду
    }
    // запускаем непрерывное обновление видеопотока
    getFrame();
  </script>
</body>
</html>
)raw";