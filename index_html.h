const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<meta charset="utf-8">
  <head>
    <title>🔒 KHÓA CỬA THÔNG MINH 🗝️</title>
     <style>
    body {
      background-color: rgb(241, 241, 241);
      padding: 10px;
    }
    .container {     
      display: flex;
      justify-content: center; 
      align-items: center; 
      flex-direction: column;
      padding-top: 10px;
      font-family: Monospace;
       
    }
    .button {
      background-color: #04AA6D; /* Green */
      border: none;
      color: white;
      padding: 15px 32px;
      text-align: center;
      text-decoration: none;
      display: inline-block;
      font-size: x-large;
      margin: 10px 2px;
      cursor: pointer;
      width: 30%;
      border-radius: 5px;
    }
    input[type=text], select {
      width: 100%;
      padding: 12px 20px;
      margin: 8px 0;
      display: inline-block;
      border: 1px solid #ccc;
      border-radius: 4px;
      box-sizing: border-box;
      font-size: xx-large;
      height : 85px;
    }
    input[type=number] {
      width: 30%;
      padding: 12px 20px;
      margin: 8px 2px;
      display: inline-block;
      border: 1px solid #ccc;
      border-radius: 4px;
      box-sizing: border-box;
      font-size: 30px;
      height : 85px;
    }

    .submit {
      width: 100%;
      background-color:#04AA6D;
      color: black;
      padding: 14px 20px;
      margin: 8px 0;
      border: none;
      border-radius: 4px;
      cursor: pointer;
    }

    .submit:hover, .button:hover {
      background-color: #989b98;
    }
    .container-2 {
      display: flex;
      margin-bottom: 20px;
      justify-content: space-between;
      gap : 10px;
    }

    h1 {
      text-align: center;
      margin-bottom: 40px;
      font-size: 50px;
    }
    h2 {
      font-size: 40px;
    }
    h4 {
      font-size: 30px;
    }
    @media (min-width: 300px) and (max-width: 900px) {
      .container {     
        margin-right: 0%;
        margin-left: 0%;
      }
      body {
        background-color: aliceblue;
      }
    }
   
  </style>
  </head>

 <body>
  <div class="container">
      <h1 > 🔒 KHÓA CỬA THÔNG MINH 🗝️</h1>
      <div>
        <hr>
        <h2>Cấu hình WIFI</h2>
        <div>
          <h4>Tên WIFI </h4>
          <input type="text" id="ssid" name="ssid" placeholder="Your ssid..">
      
          <h4>Mật khẩu</h4>
          <input type="text" id="pass" name="pass" placeholder="Your password ..">

          <h4>Mã Token Blynk</h4>
          <input type="text" id="token" name="token" placeholder="Your Token Blynk ..">
  
          <hr>

          <h2>🔒Cấu hình khóa cửa</h2>
          <div class="container-2">
            <h4>Mật khẩu 4 số</h4>
            <input  type="number" id="passDoor" name="passDoor" placeholder="0~9">
          </div>
          <div class="container-2">
            <h4>Thời gian mở cửa (giây)</h4>
            <input  type="number" id="timeOpenDoor" name="timeOpenDoor" min="1" placeholder="Giây">
          </div>
          <div class="container-2">
            <h4>Số lần cho phép nhập sai</h4>
            <input  type="number" id="numberEnterWrong" name="numberEnterWrong" min="1" placeholder="Số lần">
          </div>
          <div class="container-2">
            <h4>Thời gian khóa khi nhập sai N lần</h4>
            <input  type="number" id="timeLock" name="timeLock" min="1" placeholder="Giây">
          </div>
          
          <div class="container-2">
            <button class="submit" id="btnDefauld"><h4 style="font-size: 35px;"> Chọn mặc định</h4></button>
            <button class="submit" id="btnSubmit"><h4 style="font-size: 35px;">Gửi</h4></button>
          </div>
        </div>
      </div>
  </div>

  <script>
    var data = {
          ssid   : "",
          pass   : "",
          token : "",
          passDoor : "",
          timeOpenDoor : "",
          numberEnterWrong : "",

    };
    const ssid   = document.getElementById("ssid");
    const pass   = document.getElementById("pass");
    const token = document.getElementById("token");

    const btnDefauld = document.getElementById("btnDefauld");
    const timeOpenDoor =  document.getElementsByName('timeOpenDoor')[0];
    const numberEnterWrong =  document.getElementsByName('numberEnterWrong')[0];
    const passDoor =  document.getElementsByName('passDoor')[0];

  
    var xhttp = new XMLHttpRequest();
    xhttp.open("GET","/data_before", true),
    xhttp.send();
    xhttp.onreadystatechange = function() {
      if(xhttp.readyState == 4 && xhttp.status == 200) {
          //alert(this.responseText);
          const obj    = JSON.parse(this.responseText); // chuyển JSON sang JS Object
          //alert(obj.ssid);
          ssid.value   = obj.ssid;
          pass.value   = obj.pass;
          token.value = obj.token;
          passDoor.value = obj.passDoor;
          timeOpenDoor.value = obj.timeOpenDoor;
          numberEnterWrong.value = obj.numberEnterWrong;
          timeLock.value = obj.timeLock;
      }
    }

    // Add data mặc định vào textbox khi click vào nút btnDefauld
    btnDefauld.addEventListener("click", function(event) {
        passDoor.value = "0000";
        timeOpenDoor.value = 3;
        numberEnterWrong.value = 5;
        timeLock.value = 60;
    });

    var xhttp2 = new XMLHttpRequest();
    const btnSubmit = document.getElementById("btnSubmit"); 
    btnSubmit.addEventListener('click', () => { 
      // Biểu thức chính quy để kiểm tra giá trị
      var regex = /^\d{4}$/;

      // Kiểm tra giá trị
      if (regex.test(passDoor.value)) {
          //alert("Hợp lệ!");
          data = {
            ssid   : ssid.value,
            pass   : pass.value,
            token :  token.value,
            passDoor : Number(passDoor.value),
            timeOpenDoor : Number(timeOpenDoor.value),
            numberEnterWrong : Number(numberEnterWrong.value),
            timeLock : Number(timeLock.value),
        }
        
        xhttp2.open("POST","/post_data", true),
        xhttp2.send(JSON.stringify(data)); // chuyển JSObject sang JSON để gửi về server
        xhttp2.onreadystatechange = function() {
        if(xhttp2.readyState == 4 && xhttp2.status == 200) {
              alert("Cài đặt thành công");
          } 
        }
      } else {
          alert("Mật khẩu không hợp lệ! Hãy nhập đúng 4 ký tự số từ 0 đến 9.");
      }
        
    });

    </script>
  </body>
</html>

)rawliteral";

