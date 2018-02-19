// Stream
socket = io();

$('.stop').hide();

socket.on('liveStream', function(url_string) {
    $('#stream_image').attr('xlink:href', url_string);

    // TODO find acc and angle in image name
    var param_arr = url_string.split('_');
    var steering = parseInt(param_arr[param_arr.length-2].split('.')[0]);

    // TODO verify if correct. Here we assert many things
    var steer_to_arrow = ['35','80','125','150','175'];

    $('#dirline').attr('x2', steer_to_arrow[steering]);
    $('.start').hide();
    $('.stop').show();
});

