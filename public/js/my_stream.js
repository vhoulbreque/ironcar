// Stream
socket = io();

$('.stop').hide();

socket.on('liveStream', function(url_string) {
    $('#stream_image').attr('xlink:href', url_string);
    
    //TODO find acc and angle in image name
    var steering = 0;
    $('#dirline').attr('x2', steering);
    $('.start').hide();
    $('.stop').show();
});

/*$("#camera").click(function(event) {
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
});
*/

/*function startStream() {
    socket.emit('stream');
    console.log('start');
    $('.start').hide();
    $('.stop').show();
}

function stopStream() {
    socket.emit('stream');
    console.log('stop');
    $('.start').show();
    $('.stop').hide();
//    $('#stream').hide();
}
*/
