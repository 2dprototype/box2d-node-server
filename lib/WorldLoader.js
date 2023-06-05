const box2d = require('./box2d.min.js').box2d;

var SHAPE_BOX     = 0;
var SHAPE_CIRCLE  = 1;
var SHAPE_POLYGON = 2;
var SHAPE_CHAIN   = 3;
var SHAPE_EDGE    = 5;

var JOINT_PULLEY    = 4;
var JOINT_DISTANCE  = 0;
var JOINT_WELD      = 1;
var JOINT_REVOLUTE  = 2;
var JOINT_GEAR      = 5;
var JOINT_PRISMATIC = 6;
var JOINT_WHEEL     = 3;
var JOINT_ROPE      = 7;
var JOINT_FRICTION  = 9;
var JOINT_MOTOR     = 11;
var JOINT_MOUSE     = 10;

var SHAPE_BOX     = 0;
var SHAPE_CIRCLE  = 1;
var SHAPE_POLYGON = 2;
var SHAPE_CHAIN   = 3;
var SHAPE_EDGE    = 5;

var JOINT_PULLEY    = 4;
var JOINT_DISTANCE  = 0;
var JOINT_WELD      = 1;
var JOINT_REVOLUTE  = 2;
var JOINT_GEAR      = 5;
var JOINT_PRISMATIC = 6;
var JOINT_WHEEL     = 3;
var JOINT_ROPE      = 7;
var JOINT_FRICTION  = 9;
var JOINT_MOTOR     = 11;
var JOINT_MOUSE     = 10;

var b2Vec2 =  box2d.b2Vec2,
	b2AABB = box2d.b2AABB,
	b2BodyDef = box2d.b2BodyDef,
	b2Body = box2d.b2Body,
	b2FixtureDef = box2d.b2FixtureDef,
	b2Fixture = box2d.b2Fixture,
	b2World = box2d.b2World,
	b2MassData = box2d.b2MassData,
	b2PolygonShape = box2d.b2PolygonShape,
	b2CircleShape = box2d.b2CircleShape,
	b2DebugDraw = box2d.b2DebugDraw,
	b2MouseJointDef =  box2d.b2MouseJointDef,
	b2DistanceJointDef =  box2d.b2DistanceJointDef,
	b2RevoluteJointDef =  box2d.b2RevoluteJointDef,
	b2WeldJointDef =  box2d.b2WeldJointDef,
	b2PulleyJointDef = box2d.b2PulleyJointDef;
	b2GearJointDef = box2d.b2GearJointDef,
	b2PrismaticJointDef = box2d.b2PrismaticJointDef,
	b2MotorJointDef = box2d.b2MotorJointDef,
	b2FrictionJointDef = box2d.b2FrictionJointDef,
	particleSystemDef = box2d.b2ParticleSystemDef;



	/** for bodies with texture data */
	function GameObject () {
  		this.body;
  		this.sprite;
  		this.spriteData = [];
  		this.renderData = [];
	}
	
	function WorldLoader (){
		this.loadedBodies    = [];			// to store box2d bodies created to use when creating joints
		this.loadedJoints    = [];			// to store box2d joints created
		this.loadedParticles = [];			// to store box2d particles created
		this.gameObjects     = [];			// to store gameobjects created
		this.offsetX = 0;
		this.offsetY = 0;
	}
	
	WorldLoader.prototype.reset = function(){
		if (this.loadedBodies.length > 0){
			delete(this.loadedBodies);
		}
		this.loadedBodies = [];	
		//loadedParticles
		if (this.loadedParticles.length > 0){
			delete(this.loadedParticles);
		}
		this.loadedParticles = [];
		
		if (this.loadedJoints.length > 0){
			delete(this.loadedJoints);
		}
		this.loadedJoints = [];
		
		if (this.gameObjects.length > 0){
			delete(this.gameObjects);
		}
		this.gameObjects = [];
	};
	
	WorldLoader.prototype.loadJsonScene = function(scene, world){
		this.reset();
		
		//world.SetGravity(new b2Vec2(0,0))
		world.CreateParticleSystem(new particleSystemDef());
		// load bodies
		for (var i = 0; i < scene.bodies.length; i++){
			this.createBody(scene.bodies[i], world);
		}
		
		// load joints
		for (var i = 0; i < scene.joints.length; i++){
			this.createJoint(scene.joints[i], world);
		}
		// load particles
		for (var i = 0; i < scene.particles.length; i++){
			
			this.createParticle(scene.particles[i], world);
		}
		//load scripts
		this.customScripts(world, scene.script);
		
		var loadScene = {
							bodies    : this.loadedBodies, 
							joints    : this.loadedJoints, 
							particles : this.loadedParticles, 
							world     : world
						}
		
	    return loadScene
		
	};
	
	WorldLoader.prototype.createBody = function(b, world){
		var pB = b;
		
		var bodyDef = new b2BodyDef;
		bodyDef.type = pB.type;
		
	    bodyDef.position.x = pB.position[0] / 30
		bodyDef.position.y = pB.position[1] / 30;
		
		var body = world.CreateBody(bodyDef);
		body.SetAngle(pB.rotation * Math.PI / 180);
		body.SetBullet(pB.isBullet);
		body.SetFixedRotation(pB.isFixedRotation);
		body.SetLinearDamping(pB.linearDamping);
		body.SetAngularDamping(pB.angularDamping);
		
        body.SetAngularVelocity(pB.angularVelocity);
        body.SetLinearVelocity(pB.linearVelocity);
		body.SetAwake(pB.isAwake);
		body.SetActive(pB.isActive);
		body.SetUserData(pB.userData);
		//body.SetGravityScale(1);
		//body.SetMassData(1);
		//body.SetType(1);
		
		this.loadedBodies.push(body);
		

		if (pB.sprites.length > 0){
			this.createGameObject(pB.sprites, body);
		}
		
		for (var i = 0; i < pB.fixtures.length; i++){
			var f = pB.fixtures[i];
			var fixture = new b2FixtureDef;
			fixture.density = f.density;
			fixture.restitution = f.restitution;
			fixture.friction = f.friction;
			fixture.isSensor = f.isSensor;
			fixture.filter.maskBits = f.maskBits;
			fixture.filter.categoryBits = f.categoryBits;
			fixture.filter.groupIndex = f.groupIndex;
			fixture.userData = f.userData;
			
			for (var j = 0; j < f.shapes.length; j++){
		    	var s = f.shapes[j];
		    	if (s.type == SHAPE_BOX){
		       		var shape = new b2PolygonShape;
		       		shape.SetAsBox(s.width / 60, s.height / 60);
		       		for(var k = 0; k < shape.m_vertices.length; k++){
						shape.m_vertices[k].x += s.position[0] / 30;
						shape.m_vertices[k].y += s.position[1] / 30;
					}
		       		fixture.shape = shape;
		       		body.CreateFixture(fixture);
				}
		    	else if (s.type == SHAPE_CIRCLE){
					var shape = new b2CircleShape(s.radius * 2 / 30);
						shape.m_p.x = s.position[0] / 30;
						shape.m_p.y = s.position[1] / 30;
					
					fixture.shape = shape;
					body.CreateFixture(fixture);
				}
			    else if (s.type == SHAPE_POLYGON){
					var shape = new b2PolygonShape;
					var verts = [];
					for (var k = 0; k < s.vertices.length; k++){
						var vert = new b2Vec2(s.position[0] / 30 + s.vertices[k][0] / 30, s.position[1] / 30 + s.vertices[k][1] / 30);
						verts.push(vert); 
					}
			       	shape.Set(verts, verts.length);
			       	fixture.shape = shape;
			       	body.CreateFixture(fixture);
				}
			    else if (s.type == SHAPE_CHAIN){
					var verts = [];
					for (var k = 0; k < s.vertices.length; k++){
						var vert = new b2Vec2(s.position[0] / 30 + s.vertices[k][0] / 30, s.position[1] / 30 + s.vertices[k][1] / 30);
						verts.push(vert); 
					}
					var chain = new box2d.b2ChainShape();
                        chain.CreateChain(verts, verts.length);
			       	fixture.shape = chain;
			       	body.CreateFixture(fixture);
					}
					
				else if (s.type == SHAPE_EDGE){
					
					for(i=0;i<s.vertices.length-1;i++){
						var v1 = s.vertices[i+0];
						var v2 = s.vertices[i+1];
						/*var v0 = new box2d.b2Vec2(-0.1 ,0.0);
						var v3 = new box2d.b2Vec2(0.0, 1.0);*/
						var Vec1 = new b2Vec2( (v1[0] /30) + s.position[0] / 30, (v1[1] /30) + s.position[1] / 30 );
						var Vec2 = new b2Vec2( (v2[0] /30) + s.position[0] / 30, (v2[1]/ 30) + s.position[1] / 30 );
						
						var shape = new box2d.b2EdgeShape();
						shape.SetAsEdge(Vec1, Vec2);
					    /*shape.m_vertex0.Copy( v0 );
						shape.m_vertex3.Copy( v3 );
						shape.m_hasVertex0 = true;
						shape.m_hasVertex3 = true;*/
						fixture.shape = shape;
						body.CreateFixture(fixture);
					}	
				}
			}
		}
	};
	
	WorldLoader.prototype.createJoint = function(j, world){
		if (j.jointType == JOINT_DISTANCE){
		    var jointDef = new b2DistanceJointDef;
		    jointDef.bodyA = this.loadedBodies[j.bodyA];
		    jointDef.bodyB = this.loadedBodies[j.bodyB];
		    jointDef.localAnchorA = new b2Vec2(j.localAnchorA[0] / 30, j.localAnchorA[1] / 30);
		    jointDef.localAnchorB = new b2Vec2(j.localAnchorB[0] / 30, j.localAnchorB[1] / 30);
		    jointDef.collideConnected = j.collideConnected;
		    jointDef.length = j.length / 30;
		    jointDef.dampingRatio = j.dampingRatio;
		    jointDef.frequencyHz = j.frequencyHZ;
			var distance = world.CreateJoint(jointDef)
			distance.SetUserData(j.userData);
		    this.loadedJoints.push(distance);
		}
		else if (j.jointType == JOINT_WELD){
			var jointDef = new b2WeldJointDef;
		    jointDef.bodyA = this.loadedBodies[j.bodyA];
		    jointDef.bodyB = this.loadedBodies[j.bodyB];
		    jointDef.localAnchorA = new b2Vec2(j.localAnchorA[0] / 30, j.localAnchorA[1] / 30);
		    jointDef.localAnchorB = new b2Vec2(j.localAnchorB[0] / 30, j.localAnchorB[1] / 30);
		    jointDef.collideConnected = j.collideConnected;
		    jointDef.referenceAngle = j.referenceAngle * Math.PI / 180;
			var weld = world.CreateJoint(jointDef)
		    weld.SetUserData(j.userData);
		    this.loadedJoints.push(weld);
		}
		else if (j.jointType == JOINT_REVOLUTE){
		    var jointDef = new b2RevoluteJointDef;
		    jointDef.bodyA = this.loadedBodies[j.bodyA];
		    jointDef.bodyB = this.loadedBodies[j.bodyB];
		    jointDef.localAnchorA = new b2Vec2(j.localAnchorA[0] / 30, j.localAnchorA[1] / 30);
		    jointDef.localAnchorB = new b2Vec2(j.localAnchorB[0] / 30, j.localAnchorB[1] / 30);
		    jointDef.collideConnected = j.collideConnected;
		    jointDef.enableLimit  = j.enableLimit;
		    jointDef.enableMotor  = j.enableMotor;
		    jointDef.lowerAngle   = j.lowerAngle * Math.PI / 180;
		    jointDef.maxMotorTorque = j.maxMotorTorque;
		    jointDef.motorSpeed   = j.motorSpeed;
		    jointDef.referenceAngle = j.referenceAngle * Math.PI / 180;
		    jointDef.upperAngle   = j.upperAngle * Math.PI / 180;
			var revolute = world.CreateJoint(jointDef)
			revolute.SetUserData(j.userData)
		    this.loadedJoints.push(revolute);
		}
		else if (j.jointType == JOINT_PULLEY){
		    var jointDef = new b2PulleyJointDef;
		    jointDef.bodyA = this.loadedBodies[j.bodyA];
		    jointDef.bodyB = this.loadedBodies[j.bodyB];
		    jointDef.localAnchorA = new b2Vec2(j.localAnchorA[0] / 30, j.localAnchorA[1] / 30);
		    jointDef.localAnchorB = new b2Vec2(j.localAnchorB[0] / 30, j.localAnchorB[1] / 30);
		    jointDef.collideConnected = j.collideConnected;
		    jointDef.groundAnchorA = new b2Vec2(j.groundAnchorA[0] / 30, j.groundAnchorA[1] / 30);
		    jointDef.groundAnchorB = new b2Vec2(j.groundAnchorB[0] / 30, j.groundAnchorB[1] / 30);
		    jointDef.lengthA = j.lengthA / 30;
		    jointDef.lengthB = j.lengthB / 30;
		    jointDef.maxLengthA = j.maxLengthA / 30;
		    jointDef.maxLengthB = j.maxLengthB / 30;
		    jointDef.ratio = j.ratio;
			var pully = world.CreateJoint(jointDef)
			pully.SetUserData(j.userData)
		    this.loadedJoints.push(pully);
		}
		else if (j.jointType == JOINT_GEAR){
			var jointDef = new b2GearJointDef;
		    jointDef.bodyA = this.loadedBodies[j.bodyA];
		    jointDef.bodyB = this.loadedBodies[j.bodyB];
		    jointDef.joint1 = this.loadedJoints[j.joint1];
		    jointDef.joint2 = this.loadedJoints[j.joint2];
		    jointDef.collideConnected = j.collideConnected;
		    jointDef.ratio = j.ratio;
			var gear = world.CreateJoint(jointDef)
			gear.SetUserData(j.userData)
		    this.loadedJoints.push(gear);
		}
		else if (j.jointType == JOINT_PRISMATIC){
			var jointDef = new b2PrismaticJointDef;
		    jointDef.bodyA = this.loadedBodies[j.bodyA];
		    jointDef.bodyB = this.loadedBodies[j.bodyB];
		    jointDef.localAnchorA = new b2Vec2(j.localAnchorA[0] / 30, j.localAnchorA[1] / 30);
		    jointDef.localAnchorB = new b2Vec2(j.localAnchorB[0] / 30, j.localAnchorB[1] / 30);
		    jointDef.localAxisA = new b2Vec2(j.localAxisA[0], j.localAxisA[1]);
		    jointDef.collideConnected = j.collideConnected;
		    jointDef.enableLimit  = j.enableLimit;
		    jointDef.enableMotor  = j.enableMotor;
		    jointDef.lowerTranslation   = j.lowerTranslation / 30;
		    jointDef.maxMotorForce = j.maxMotorForce;
		    jointDef.motorSpeed   = j.motorSpeed;
		    jointDef.referenceAngle = j.referenceAngle * Math.PI / 180;
		    jointDef.upperTranslation   = j.upperTranslation / 30;
			var prismatic = world.CreateJoint(jointDef)
			prismatic.SetUserData(j.userData)
		    this.loadedJoints.push(prismatic);
		}
		
	 	// not supported box2d-web BUT supported in box2d.min :)
	 	else if (j.jointType == JOINT_WHEEL){
			var axis = new b2Vec2(j.localAxisA[0], j.localAxisA[1]);
			var jd = new box2d.b2WheelJointDef();
			jd.Initialize(this.loadedBodies[j.bodyA], this.loadedBodies[j.bodyB], this.loadedBodies[j.bodyB].GetPosition(), axis);
			jd.collideConnected = j.collideConnected;
			jd.motorSpeed = j.motorSpeed;
			jd.maxMotorTorque = j.maxMotorTorque;
			jd.enableMotor = j.enableMotor;
			jd.frequencyHz = j.frequencyHZ;
			jd.dampingRatio = j.dampingRatio;
		    jd.localAnchorA = new b2Vec2(j.localAnchorA[0] / 30, j.localAnchorA[1] / 30);
		    jd.localAnchorB = new b2Vec2(j.localAnchorB[0] / 30, j.localAnchorB[1] / 30);
			jd.localAxisA = new b2Vec2(j.localAxisA[0], j.localAxisA[1]);
			var wheel = world.CreateJoint(jd)
			wheel.SetUserData(j.userData)
		    this.loadedJoints.push(wheel);
		}
		
		// not supported box2d-web BUT supported in box2d.min :)
		else if (j.jointType == JOINT_ROPE){
			var ropeJointDef = new box2d.b2RopeJointDef();
		    ropeJointDef.localAnchorA = new b2Vec2(j.localAnchorA[0] / 30, j.localAnchorA[1] / 30);
		    ropeJointDef.localAnchorB = new b2Vec2(j.localAnchorB[0] / 30, j.localAnchorB[1] / 30);
			ropeJointDef.bodyA = this.loadedBodies[j.bodyA];
		    ropeJointDef.bodyB = this.loadedBodies[j.bodyB];
			ropeJointDef.maxLength = j.maxLength / 30;
			ropeJointDef.collideConnected = j.collideConnected;
			var rope = world.CreateJoint(ropeJointDef);
			rope.SetUserData(j.userData)
		    this.loadedJoints.push(rope);
		}
		else if (j.jointType == JOINT_FRICTION){
			
			var f = new b2FrictionJointDef();
			f.Initialize(this.loadedBodies[j.bodyA],this.loadedBodies[j.bodyB], this.loadedBodies[j.bodyB].GetPosition())
			f.maxForce = j.maxForce;
			f.maxTorque = j.maxTorque;
			f.collideConnected = j.collideConnected;
		    f.localAnchorA = new b2Vec2(j.localAnchorA[0] / 30, j.localAnchorA[1] / 30);
		    f.localAnchorB = new b2Vec2(j.localAnchorB[0] / 30, j.localAnchorB[1] / 30);
			var friction = world.CreateJoint(f);
		    friction.SetUserData(j.userData)
		    this.loadedJoints.push(friction);

		}
		else if (j.jointType == JOINT_MOTOR){
			
		var m = new b2MotorJointDef();
		    m.Initialize(this.loadedBodies[j.bodyA],this.loadedBodies[j.bodyB]);
			m.maxForce = j.maxForce;
			m.maxTorque = j.maxTorque;
			m.collideConnected = j.collideConnected;
			m.linearOffset = new b2Vec2(j.linearOffset[0], j.linearOffset[1]);
			m.angularOffset = j.angularOffset;
			m.correctionFactor = j.correctionFactor;
			var motor = world.CreateJoint(m);
			motor.SetUserData(j.userData)
		    this.loadedJoints.push(motor);
		}
		else if (j.jointType == JOINT_MOUSE){
		var md = new b2MouseJointDef();
			md.bodyA = world.CreateBody(new box2d.b2BodyDef());
			md.bodyB = this.loadedBodies[j.bodyB];
			md.target = new b2Vec2(j.target[0] / 30, j.target[1] / 30);
			md.collideConnected = j.collideConnected;
			md.maxForce = j.maxForce;
		    md.dampingRatio = j.dampingRatio;
		    md.frequencyHz = j.frequencyHZ;
			var mouse = world.CreateJoint(md);
			mouse.SetTarget(new b2Vec2(j.groundBody[0] / 30, j.groundBody[1] / 30));
			mouse.m_userData = j.userData;
		    this.loadedJoints.push(mouse);
		}
		else if (j.jointType == Joint.JOINT_AREA){
			var aj = new box2d.b2AreaJointDef();
			aj.world = world;
			aj.dampingRatio = j.dampingRatio;
			aj.frequencyHz = j.frequencyHZ;
			aj.collideConnected = j.collideConnected;
			for( b = 0; b < j.bodies.length; b++ ){
				aj.AddBody(this.loadedBodies[j.bodies[b]]);
			}
			var	area = world.CreateJoint(aj);
			area.m_userData = j.userData;
			this.loadedJoints.push(area);
		}
	};
	
	
	WorldLoader.prototype.createParticle = function(p, world){

		if(p.type == 0){ 
			var shape = new box2d.b2CircleShape( (p.shape.radius) / 30) 
		}
		else if(p.type == 1){ 
			var shape = new box2d.b2PolygonShape();
			shape.SetAsBox((p.shape.width) / 60, (p.shape.height) / 60);
		}
		else if(p.type == 2){ 
		
			var shape = new box2d.b2PolygonShape();
			var verts = [];
			var s = p.shape;
			for (var k = 0; k < s.vertices.length; k++){
				var vert = new b2Vec2(s.vertices[k][0] / 30, s.vertices[k][1] / 30);
				verts.push(vert); 
			}
		    shape.Set(verts, verts.length);
		}
		
		var pd = new box2d.b2ParticleGroupDef();
		pd.position  = new b2Vec2(p.position[0] / 30, p.position[1] / 30)
		pd.angle = (p.rotation * Math.PI / 180);
		pd.color = p.color;
		pd.flags = p.flags;
		pd.groupFlags = box2d.b2ParticleGroupFlag.b2_solidParticleGroup;
		pd.shape = shape;
		pd.strength = p.strength
		pd.stride = p.stride
		pd.linearVelocity = new b2Vec2(p.linearVelocity[0], p.linearVelocity[1]);
		pd.angularVelocity = p.angularVelocity;
		pd.lifetime = p.lifetime;
		//pd.stride = 1;
		
		world.GetParticleSystemList().SetRadius(p.radius)
		this.loadedParticles.push( world.GetParticleSystemList().CreateParticleGroup(pd) );
		
	}
	
	
	WorldLoader.prototype.customScripts = function(m_world, script){
		var world     = m_world;
		var Bodies    = this.loadedBodies;
		var Joints    = this.loadedJoints;
		var Particles = this.loadedParticles;

		try {
			eval(script);
		}
		catch(err) {
			console.error("WorldLoader : "+ err.message);
			CS.error("WorldLoader : "+ err.message);
			Editor.gameView.paused = true;
		}
		
		
	}
	
	WorldLoader.prototype.createGameObject = function(s, body){
		
		var gameObj = [];
		
		for(i=0;i<s.length;i++){
			var gO = new GameObject();
			gO.sprite = new Image();
			gO.sprite.src = Editor.sceneManager.loadedFile.dir + s[i].src;
			gO.body = body;
			gO.spriteData = [s[i].width,s[i].height,s[i].position[0],s[i].position[1],s[i].rotation]; //[width, height, position-x, position-y, rotation]
			gO.renderData = [s[i].flip,s[i].opacity]; //[Flip, opacity]
			
			gameObj.push(gO)
		}
		
		this.gameObjects.push(gameObj);
	};

module.exports = new WorldLoader;