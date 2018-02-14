// Stream
socket = io();

$('.stop').hide();

socket.on('liveStream', function(url) {
    $('#stream').attr('src', url);
    $('.start').hide();
    $('.stop').show();
});

function startStream() {
    socket.emit('start-stream');
    console.log('start');
    $('.start').hide();
    $('.stop').show();
}

function stopStream() {
    socket.emit('stop-stream');
    console.log('stop');
    $('.start').show();
    $('.stop').hide();
//    $('#stream').hide();
}
