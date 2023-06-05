const express = require('express');
const app = express();
const server = require('http').Server(app);
const io = require('socket.io')(server,{});
const box2d = require('./lib/box2d.min.js').box2d;
const WorldLoader = require('./lib/WorldLoader.js');
const fs = require('fs');

//hosting
const port = 2000;
 
app.get('/',function(req, res) {
	res.sendFile(__dirname + '/client/index.html');
});
app.use('/', express.static(__dirname + '/client'));
 
server.listen(port, function(){
    console.log(`Server is listening on port ${port}`);
    console.log(`localhost:${port}`);
});





//game
var json_file = fs.readFileSync("scenes/scene_0_exported.json", 'utf8');
var scene = JSON.parse(json_file);
var world = new box2d.b2World(new box2d.b2Vec2(0, 10), true);
world.ClearForces();
WorldLoader.loadJsonScene(scene, world);


function get_data(scene){
	function c_body(){
		this.position = { x : 0, y : 0 };
		this.angle;
		this.shapes;
	}
	function c_shape(){
		this.type;
	}
	
	var arr_b = []
	var b = scene.loadedBodies;
	for(i = 0; i < b.length; i++){
		var body = b[i];
		var cb = new c_body();
		cb.position.x = body.GetPosition().x;
		cb.position.y = body.GetPosition().y;
		cb.angle = b[i].GetAngle();
		
		var __shapes = [];
		
		for (var f = body.GetFixtureList(); f !== null; f = f.GetNext()){
			var shape = f.m_shape;
			var cs = new c_shape();
			if(shape.m_type == box2d.b2ShapeType.e_circleShape){
				cs.type = 'circle';
				cs.radius = shape.m_radius;
				cs.p = {};
				cs.p.x = shape.m_p.x;
				cs.p.y = shape.m_p.y;
			}
			else if(shape.m_type == box2d.b2ShapeType.e_polygonShape){
				cs.type = 'polygon';
				cs.vertices = shape.m_vertices;
				cs.vertexCount = shape.m_count;
			}
			__shapes.push(cs)
			
		}
		cb.shapes = __shapes
		
		arr_b.push(cb)
	}
	return { bodies : arr_b , joints : [] }
}




function random_string(length) {
    let result = ' ';
	var characters ='ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789';
    var charactersLength = characters.length;
    for ( let i = 0; i < length; i++ ) {
        result += characters.charAt(Math.floor(Math.random() * charactersLength));
    }

    return result;
}

var SOCKET_LIST = {};

io.sockets.on('connection', function(socket){
	socket.id = random_string(7);
	socket.x = 0;
	socket.y = 0;
	socket.number = socket.id;
	SOCKET_LIST[socket.id] = socket;

	socket.on('disconnect',function(){
		delete SOCKET_LIST[socket.id];
	});
});



var update = function(){
	world.Step(1 / 60, 10, 10);
	// io.sockets.on('connection', function(socket){
	 
		// world.Step(1 / 60, 10, 10);
		
		// socket.emit('send_data',{
			// scene : get_data(WorldLoader)
		// });
	 
	// });
	var pack = [];
	for(var i in SOCKET_LIST){
		var socket = SOCKET_LIST[i];
		socket.x = Math.random()*50;
		socket.y = Math.random()*50;
		pack.push({
			x:socket.x,
			y:socket.y,
			number:socket.number
		});		
	}
	for(var i in SOCKET_LIST){
		var socket = SOCKET_LIST[i];
		socket.emit('send_data',{
			pack : pack,
			scene : get_data(WorldLoader)
		});
	}
}

setInterval(update, 1000/60);

