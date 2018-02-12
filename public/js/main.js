var socket = io.connect();


// -------- MODE ------
$("[data-mode]").click(function(event) {
  event.preventDefault();
  var mode = $(this).data('mode');
  $("[data-mode]").each(function() {
    if($(this).hasClass('btn-primary'))
        $(this).toggleClass('btn-primary btn-outline-primary');
  });
  $("[data-mode]").removeClass('btn-primary');
  $(this).toggleClass('btn-outline-primary btn-primary');
  console.log(mode);
  socket.emit("modeSwitched", mode);
});


// -------- KEYBOARD INPUT -----------
kinput.onkeydown = kinput.onkeyup = kinput.onkeypress = handle;

function handle(e) {
    // Gas control
    if (e.key == "ArrowDown" && e.type == "keydown" && !e.repeat){ socket.emit("gas", -1);}
    if (e.key == "ArrowUp" && e.type == "keydown" && !e.repeat){ socket.emit("gas", 1);}
    if (e.key == "ArrowUp" && e.type == "keyup" && !e.repeat){ socket.emit("gas", 0);}
    if (e.key == "ArrowDown" && e.type == "keyup" && !e.repeat){ socket.emit("gas", 0);}

    // Direction control
    if (e.key == "ArrowLeft" && e.type == "keydown" && !e.repeat){ socket.emit("dir", -1);}
    if (e.key == "ArrowRight" && e.type == "keydown" && !e.repeat){ socket.emit("dir", 1);}
    if (e.key == "ArrowLeft" && e.type == "keyup" && !e.repeat){ socket.emit("dir", 0);}
    if (e.key == "ArrowRight" && e.type == "keyup" && !e.repeat){ socket.emit("dir", 0);}

}

// -------- MAX SPEED UPDATE -----------

function maxSpeedUdate(){
    var newMaxSpeed = document.getElementById("maxSpeedSlider").value ;
    document.getElementById("maxSpeed").innerHTML = "Max speed limit: " + newMaxSpeed + "%";
    socket.emit("maxSpeed", newMaxSpeed / 100.);
}

// update the current max speed
socket.on('maxSpeedUpdate', function(maxSpeed){
    document.getElementById("maxSpeed").innerHTML = "Max speed limit: " + Math.round(maxSpeed * 100) + "%";
    document.getElementById("maxSpeedSlider").value = maxSpeed * 100;
});


// -------- STARTER -----------

$("#starter").click(function( event ) {
  event.preventDefault();
  console.log('starter');
  socket.emit('starter');
  $(this).toggleClass('btn-success btn-danger');
});

socket.on('starterUpdate', function(data){
    var state = 'Stop';
    if (data == "stopped"){
        state = 'Start';
    }
    $("#starter").html(state);
});


// -------- AUTOPILOT MODEL -----------

socket.on('new_available_model', function(modelList){
    console.log(modelList);
    var mySelect = $("#model_select");
    var options_html = "<option selected>Choose model...</option>";
    for (var i = 0; i < modelList.length; i++) {
        options_html += '<option>';
        options_html += modelList[i];
        options_html += '</option>';
    }
    mySelect.html(options_html);
});


$( "#model_select" ).change(function() {
    var modelName = $(this).val();
    console.log(modelName);
    if(modelName != "Choose model...")
        socket.emit('model_update', modelName);
});

socket.on("model_update", function(modelSelected){
    var mySelect = document.getElementById("model_select");
    for (var i = 0; i < mySelect.options.length; i++) {
        if (modelSelected == mySelect.options[i].text){
            var modelIndex = i;
        }
    }
    mySelect.selectedIndex = modelIndex;
});

// -------- USER INFO -----------

// Message to the user
socket.on('msg2user', function(message){
    $("#Status").text(message);
});

socket.emit('clientLoadedPage', true);