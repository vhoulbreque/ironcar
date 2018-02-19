var socket = io.connect();

$(document).ready( function() {
    $('#model-group').hide();
    $('#control-group').show();
});

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

    if (mode == 'training') {
        $('#model-group').hide();
        $('#control-group').show();
        $('#starter').prop("disabled",false);
    }
    else if (mode == 'rest') {
        $('#model-group').hide();
        $('#control-group').hide();
        $('#starter').prop("disabled",false);
    }
    else {
        $('#model-group').show();
        $('#control-group').hide();
        // TODO disable if model loaded
        $('#starter').prop("disabled",false);
    }

    

});


// -------- KEYBOARD INPUT -----------
kinput.onkeydown = kinput.onkeyup = kinput.onkeypress = handle;

function handle(e) {

    var elem = $("#control");

    // Gas control
    if (e.key == "ArrowDown" && e.type == "keydown" && !e.repeat){elem.removeClass().addClass('oi oi-caret-bottom'); socket.emit("gas", -1);}
    if (e.key == "ArrowUp" && e.type == "keydown" && !e.repeat){elem.removeClass().addClass('oi oi-caret-top'); socket.emit("gas", 1);}
    if (e.key == "ArrowUp" && e.type == "keyup" && !e.repeat){elem.removeClass().addClass('oi oi-media-pause'); socket.emit("gas", 0);}
    if (e.key == "ArrowDown" && e.type == "keyup" && !e.repeat){elem.removeClass().addClass('oi oi-media-pause'); socket.emit("gas", 0);}

    // Directoin control
    if (e.key == "ArrowLeft" && e.type == "keydown" && !e.repeat){elem.removeClass().addClass('oi oi-caret-left'); socket.emit("dir", -1);}
    if (e.key == "ArrowRight" && e.type == "keydown" && !e.repeat){elem.removeClass().addClass('oi oi-caret-right'); socket.emit("dir", 1);}
    if (e.key == "ArrowLeft" && e.type == "keyup" && !e.repeat){elem.removeClass().addClass('oi oi-media-pause'); socket.emit("dir", 0);}
    if (e.key == "ArrowRight" && e.type == "keyup" && !e.repeat){elem.removeClass().addClass('oi oi-media-pause'); socket.emit("dir", 0);}

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
});

socket.on('starterUpdate', function(data){
    console.log('ici');
    var state = 'Stop';
    if (data == "stopped"){
        state = 'Start';
        $('[data-mode').prop("disabled",false);
        $("#starter").removeClass('btn-danger').addClass('btn-success');
    } else {
        $('[data-mode').prop("disabled",true);
        $("#starter").removeClass('btn-success').addClass('btn-danger');
    }

    $("#starter").html(state);
});


// --------- START CAMERA ---------

$("#camera").click(function(event) {
    event.preventDefault();
    console.log('toggle camera');
    socket.emit('streamUpdate');
    $(this).toggleClass('btn-success btn-danger');

});

socket.on('stream', function(data) {
    state = "Stop camera";
    if (data == "stopped") {
        state = "Start camera";
    }
    $("#camera").html(state);
});


// -------- AUTOPILOT MODEL -----------

socket.on('new_available_model', function(modelList){
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
