var socket = io();
var canvas = document.getElementById("ctx");
var width = canvas.width;
var hight = canvas.height;
var ctx = canvas.getContext("2d");
ctx.font = '10px Arial';


socket.on('send_data',function(data){
	ctx.clearRect(-width, -hight, width*2, hight*2);
	ctx.fillStyle = '#3e3f35';
	//ctx.fillStyle = '#f7f7f7';
	ctx.fillRect(-width, -hight, width*2, hight*2);
	var pack = data.pack;
	for(var i = 0 ; i < pack.length; i++){
		ctx.fillStyle = '#4b963a';
		ctx.fillText(pack[i].number,pack[i].x,pack[i].y)
	}
	var scene = data.scene;
	var scale = 30;
	var w = 5;
	var h = 5;
	for(var b = 0 ; b < scene.bodies.length; b++){
		var body = scene.bodies[b];
		for(f = 0; f < body.shapes.length; f++){
			draw_shape(ctx, scale, body, body.shapes[f])
		}
	}
});

function draw_shape(context, scale, body, shape){
	context.save();
	context.scale(scale,scale);
	// context.globalAlpha = 0.5;
	// context.fillStyle = '#7fc728';
	context.strokeStyle = "#4b963a";
	context.translate(body.position.x, body.position.y);
	context.rotate(body.angle);
	
	context.beginPath();
	context.lineWidth /= scale;
	
	switch(shape.type) {
		case 'circle' : {
			var r = shape.radius;
			context.translate(shape.p.x, shape.p.y);
			context.arc(0, 0, r, 0, 2 * Math.PI, false);
			context.moveTo(0, 0);
			context.lineTo(r, 0);
			// context.fill();
			
		} break;
		
		case 'polygon' :{
			
			var vertices = shape.vertices;
			var vertexCount = shape.vertexCount;
			if (!vertexCount) return;
			
			context.moveTo(vertices[0].x, vertices[0].y);
			for (var i = 0; i < vertexCount; i++)
			context.lineTo(vertices[i].x, vertices[i].y);
			
			// context.fill();
		} break;
		
	}
	
	context.closePath();
	context.stroke();
	
	context.restore();
}











// socket.on('go',function(data){
// console.log(data)
// });

// function draw_body(bodies,ctx){
// for(i = 0; i < bodies.length; i++){
// var x = bodies[i].x;
// var y = bodies[i].y;
// var a = bodies[i].angle;
// ctx.fillText("Hi",x,y);
// }
// }

