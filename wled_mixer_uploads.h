
WebServer server(80);

// modified version of https://community.appinventor.mit.edu/t/esp32-wifi-webserver-upload-file-from-app-to-esp32-sdcard-reader-littlefs/28126/2
String serverIndex = "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
    "<input type='file' name='update'>"
    "<input type='submit' value='Upload'>"
"</form>"
"<div id='prg'>progress: 0%</div>"
"<script>"
"$('form').submit(function(e){"
    "e.preventDefault();"
      "var form = $('#upload_form')[0];"
      "var data = new FormData(form);"
      " $.ajax({"
            "url: '/update',"
            "type: 'POST',"               
            "data: data,"
            "contentType: false,"                  
            "processData:false,"  
            "xhr: function() {"
                "var xhr = new window.XMLHttpRequest();"
                "xhr.upload.addEventListener('progress', function(evt) {"
                    "if (evt.lengthComputable) {"
                        "var per = evt.loaded / evt.total;"
                        "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
                    "}"
               "}, false);"
               "return xhr;"
            "},"                                
            "success:function(d, s) {"    
                "console.log('success!')"
           "},"
            "error: function (a, b, c) {"
            "}"
          "});"
"});"
"</script>";

File root;
bool opened = false;

String printDirectory(File dir) {
  String response = "";
  dir.rewindDirectory();
  
  while(true) {
     File entry =  dir.openNextFile();
     if (! entry) break;  // no more files
     response += String("<a href='") + String(entry.name()) + String("'>") + String(entry.name()) + "\t\tsize: " + String(entry.size()) + String("</a>") + String("</br>");
     entry.close();
  }
  return String("List files:</br>") + response + String("</br></br> Upload file:") + serverIndex;
}

void handleRoot() {
  root = SPIFFS.open("/");
  String res = printDirectory(root);
  server.send(200, "text/html", res);
}


void handleNotFound(){
  Serial.println(" Not Found ");
}

void handleUpdate(){
  HTTPUpload& upload = server.upload();
  if (opened == false){
    opened = true;
    root = SPIFFS.open((String("/") + upload.filename).c_str(), FILE_WRITE); 
  
    if(!root){
      Serial.println("- failed to open file for writing");
      return;
    }
  } 
  if(upload.status == UPLOAD_FILE_WRITE){
    if(root.write(upload.buf, upload.currentSize) != upload.currentSize){
      Serial.println("- failed to write");
      return;
    }
  } else if(upload.status == UPLOAD_FILE_END){
    root.close();
    Serial.println("UPLOAD_FILE_END");
    opened = false;
  }  
}
