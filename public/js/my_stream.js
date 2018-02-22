// Stream
socket = io();

$('.stop').hide();

socket.on('picture_stream', function(data) {
    // data = { image: true, buffer: img_base64, index: index_class}
    if (data.image) {
        $('#stream_image').attr('xlink:href', 'data:image/jpeg;base64,'+data.buffer);

        // TODO find acc and angle in image name
        // TODO verify if correct. Here we assert many things
        var steer_to_arrow = ['35','80','-1','150','175'];
        if (steer_to_arrow == '-1') {
        	$('#dirline').attr('visibility', 'hidden');
        } else {
        	$('#dirline').attr('visibility', 'visible');
    	    $('#dirline').attr('x2', steer_to_arrow[data.index]);
        }
        $('.start').hide();
        $('.stop').show();
    }
});

