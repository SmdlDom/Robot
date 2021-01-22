
THREE.Object3D.prototype.setMatrix = function(a) {
  this.matrix = a;
  this.matrix.decompose(this.position, this.quaternion, this.scale);
};

var start = Date.now();
// SETUP RENDERER AND SCENE
var scene = new THREE.Scene();
var renderer = new THREE.WebGLRenderer();
renderer.setClearColor(0xffffff); // white background colour
document.body.appendChild(renderer.domElement);

// SETUP CAMERA
var camera = new THREE.PerspectiveCamera(30, 1, 0.1, 1000); // view angle, aspect ratio, near, far
camera.position.set(10,5,10);
camera.lookAt(scene.position);
scene.add(camera);

// SETUP ORBIT CONTROL OF THE CAMERA
var controls = new THREE.OrbitControls(camera);
controls.damping = 0.2;

// ADAPT TO WINDOW RESIZE
function resize() {
  renderer.setSize(window.innerWidth, window.innerHeight);
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
}

window.addEventListener('resize', resize);
resize();

// FLOOR WITH CHECKERBOARD
var floorTexture = new THREE.ImageUtils.loadTexture('tile.jpg');
floorTexture.wrapS = floorTexture.wrapT = THREE.MirroredRepeatWrapping;
floorTexture.repeat.set(4, 4);

var floorMaterial = new THREE.MeshBasicMaterial({ map: floorTexture, side: THREE.DoubleSide });
var floorGeometry = new THREE.PlaneBufferGeometry(15, 15);
var floor = new THREE.Mesh(floorGeometry, floorMaterial);
floor.rotation.x = Math.PI / 2;
floor.position.y = 0.0;
scene.add(floor);

// TRANSFORMATIONS

function multMat(m1, m2){
  return new THREE.Matrix4().multiplyMatrices(m1, m2);
}

function inverseMat(m){
  return new THREE.Matrix4().getInverse(m, true);
}

function multVecByMat(v,m){

  var res = new THREE.Vector3()
  res.x = v.x*m.elements[0]+v.y*m.elements[3]+v.z*m.elements[6];
  res.y = v.x*m.elements[1]+v.y*m.elements[4]+v.z*m.elements[7];
  res.z = v.x*m.elements[2]+v.y*m.elements[5]+v.z*m.elements[8];
  return res;
}

function idMat4(){
  var m = new THREE.Matrix4();

  m.set(1,0,0,0,
      0,1,0,0,
      0,0,1,0,
      0,0,0,1);
  return m;
}

function translateMat(matrix, x, y, z){
  var translationM = new THREE.Matrix4();

  translationM.set(1,0,0,x,
      0,1,0,y,
      0,0,1,z,
      0,0,0,1);

  return multMat(matrix,translationM);
}

function rotateMat(matrix, angle, axis) {
  var rotationM = new THREE.Matrix4();

  switch (axis) {
    case "x":
      rotationM.set(1, 0, 0, 0,
          0, Math.cos(angle), -Math.sin(angle), 0,
          0, Math.sin(angle), Math.cos(angle), 0,
          0, 0, 0, 1);
      break;
    case "y":
      rotationM.set(Math.cos(angle), 0, Math.sin(angle), 0,
          0, 1, 0, 0,
          -Math.sin(angle), 0, Math.cos(angle), 0,
          0, 0, 0, 1);
      break;
    case "z":
      rotationM.set(Math.cos(angle), -Math.sin(angle), 0, 0,
          Math.sin(angle), Math.cos(angle), 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1);
      break;
  }

  return multMat(matrix,rotationM);
}

function rotateVec3(v, angle, axis){
  var rotationM = new THREE.Matrix3();

  switch(axis) {
    case "x":
      rotationM.set(1,0,0,
          0,Math.cos(angle),-Math.sin(angle),
          0,Math.sin(angle),Math.cos(angle));
      break;
    case "y":
      rotationM.set(Math.cos(angle),0,Math.sin(angle),
          0,1,0,
          -Math.sin(angle),0,Math.cos(angle));
      break;
    case "z":
      rotationM.set(Math.cos(angle),-Math.sin(angle),0,
          Math.sin(angle),Math.cos(angle),0,
          0,0,1);
      break;
  }
  return multVecByMat(v,rotationM);
}

function rescaleMat(matrix, x, y, z){
  var scaleM = new THREE.Matrix4();

  scaleM.set(x,0,0,0,
      0,y,0,0,
      0,0,z,0,
      0,0,0,1)

  return multMat(matrix,scaleM);
}

class Robot {
  constructor() {
    // Geometry
    this.torsoHeight = 1.5;
    this.torsoRadius = 0.75;
    this.headRadius = 0.32;
    this.armLengthRescale = 2.5;
    this.armRadius = 0.15;
    this.gap = 0.06;
    this.forearmLengthRescale = 8;
    this.forearmRadius = 0.075;
    this.thighLengthRescale= 3;
    this.thighRadius = 0.32
    this.calfLengthRescale = 5;
    this.calfRadius = 0.1;
    this.sphereNumSeg = 20;

    // Animation
    this.walkDirection = new THREE.Vector3( 0, 0, 1 );
    this.walking = false;
    this.walkFrame = 0;
    this.walkFrameCap = 80;
    this.fwdThighAngleVar = 0.04;
    this.bwdThighAngleVar = 0.02;
    this.fwdCalfAngleVar = 0.02;
    this.bwdCalfAngleVar = 0;

    // Material
    this.material = new THREE.MeshNormalMaterial();

    // Initial pose
    this.initialize()
  }

  resetWalkingField() {
    this.walking = false;
    this.walkFrame = 0;
  }

  initialMatrix(lr, x, y, z) {
    //lr : string "l" or "r"
    var matrix = idMat4();
    if (lr === "l") {
      matrix = translateMat(matrix,x,y,z);
    } else {
      matrix = translateMat(matrix,-x,y,z);
    }
    return matrix;
  }

  initialTorsoMatrix(){
    return this.initialMatrix("l", 0, this.torsoHeight/2 + this.thighRadius*this.thighLengthRescale*2
        + this.calfRadius*this.calfLengthRescale*2 + this.gap*2, 0);
  }

  initialHeadMatrix(){
    return this.initialMatrix("l", 0, this.torsoHeight/2 + this.headRadius, 0);
  }

  initialArmMatrix(lr){
    return this.initialMatrix(lr, this.torsoRadius+this.armRadius, this.torsoHeight/2,0);
  }

  initialForearmMatrix(){
    return this.initialMatrix("l", 0, -this.armRadius*this.armLengthRescale*2-this.gap,0);
  }

  initialThighMatrix(lr){
    return this.initialMatrix(lr, this.torsoRadius/2, -this.torsoHeight/2, 0);
  }

  initialCalfMatrix(){
    return this.initialMatrix("l", 0, -this.thighLengthRescale*this.thighRadius*2-this.gap*2,0);
  }

  genSphereMesh(radius) {
    var geo = new THREE.SphereGeometry(radius, this.sphereNumSeg ,this.sphereNumSeg);
    return new THREE.Mesh(geo, this.material);
  }

  transformMat(mat, translation, rescale) {
    mat = translateMat(mat,0,translation,0);
    mat = rescaleMat(mat,1,rescale,1);
    return mat;
  }

  genFirstOrderPos(mat, matInit) {
    return multMat(multMat(this.torsoInitialMatrix, matInit), mat);
  }

  genSecondOrderPos(mat, matInit, linkInit) {
    return multMat(multMat(multMat(this.torsoInitialMatrix,linkInit),matInit),mat);
  }

  initialize() {
    var torsoGeometry = new THREE.CubeGeometry(2*this.torsoRadius, this.torsoHeight, this.torsoRadius, 64);
    this.torso = new THREE.Mesh(torsoGeometry, this.material);

    var headGeometry = new THREE.CubeGeometry(2*this.headRadius, this.headRadius, this.headRadius);
    this.head = new THREE.Mesh(headGeometry, this.material);

    this.leftArm = this.genSphereMesh(this.armRadius);
    this.rightArm = this.genSphereMesh(this.armRadius);
    this.leftForearm = this.genSphereMesh(this.forearmRadius);
    this.rightForearm = this.genSphereMesh(this.forearmRadius);
    this.leftThigh = this.genSphereMesh(this.thighRadius);
    this.rightThigh = this.genSphereMesh(this.thighRadius);
    this.leftCalf = this.genSphereMesh(this.calfRadius);
    this.rightCalf = this.genSphereMesh(this.calfRadius);

    // Torse transformation
    this.torsoInitialMatrix = this.initialTorsoMatrix();
    this.torsoMatrix = idMat4();
    this.torso.setMatrix(this.torsoInitialMatrix);

    // Head transformation
    this.headInitialMatrix = this.initialHeadMatrix();
    this.headMatrix = idMat4();
    this.head.setMatrix(this.genFirstOrderPos(this.headMatrix,this.headInitialMatrix));

    // Left Arm Transformation
    this.leftArmInitialMatrix = this.initialArmMatrix("l");
    this.leftArmMatrix = idMat4();
    this.leftArmRotation = idMat4();
    this.leftArmMatrix = this.transformMat(this.leftArmMatrix, -this.armRadius*this.armLengthRescale,
        this.armLengthRescale );
    this.leftArm.setMatrix(this.genFirstOrderPos(this.leftArmMatrix,this.leftArmInitialMatrix));

    // Right Arm Transformation
    this.rightArmInitialMatrix = this.initialArmMatrix("r");
    this.rightArmMatrix = idMat4();
    this.rightArmRotation = idMat4();
    this.rightArmMatrix = this.transformMat(this.rightArmMatrix, -this.armRadius*this.armLengthRescale,
        this.armLengthRescale );
    this.rightArm.setMatrix(this.genFirstOrderPos(this.rightArmMatrix,this.rightArmInitialMatrix));

    // Left Forearm Transformation
    this.leftForearmInitialMatrix = this.initialForearmMatrix();
    this.leftForearmMatrix = idMat4();
    this.leftForearmMatrix = this.transformMat(this.leftForearmMatrix, -this.forearmRadius*this.forearmLengthRescale,
        this.forearmLengthRescale );
    this.leftForearm.setMatrix(this.genSecondOrderPos(this.leftForearmMatrix, this.leftForearmInitialMatrix,
        this.leftArmInitialMatrix));

    // Right Forearm Transformation
    this.rightForearmInitialMatrix = this.initialForearmMatrix();
    this.rightForearmMatrix = idMat4();
    this.rightForearmMatrix = this.transformMat(this.rightForearmMatrix, -this.forearmRadius*this.forearmLengthRescale,
        this.forearmLengthRescale );
    this.rightForearm.setMatrix(this.genSecondOrderPos(this.rightForearmMatrix, this.rightForearmInitialMatrix,
        this.rightArmInitialMatrix));

    // Left Thigh Transformation
    this.leftThighInitialMatrix = this.initialThighMatrix("l");
    this.leftThighMatrix = idMat4();
    this.leftThighRotation = idMat4();
    this.leftThighMatrix = this.transformMat(this.leftThighMatrix, -this.thighLengthRescale*this.thighRadius-this.gap,
        this.thighLengthRescale );
    this.leftThigh.setMatrix(this.genFirstOrderPos(this.leftThighMatrix,this.leftThighInitialMatrix));

    // Right Thigh Transformation
    this.rightThighInitialMatrix = this.initialThighMatrix("r");
    this.rightThighMatrix = idMat4();
    this.rightThighRotation = idMat4();
    this.rightThighMatrix = this.transformMat(this.rightThighMatrix, -this.thighLengthRescale*this.thighRadius-this.gap,
        this.thighLengthRescale );
    this.rightThigh.setMatrix(this.genFirstOrderPos(this.rightThighMatrix,this.rightThighInitialMatrix));

    // Left calf Transformation
    this.leftCalfInitialMatrix = this.initialCalfMatrix();
    this.leftCalfMatrix = idMat4();
    this.leftCalfRotation = idMat4();
    this.leftCalfMatrix = this.transformMat(this.leftCalfMatrix, -this.calfRadius*this.calfLengthRescale,
        this.calfLengthRescale);
    this.leftCalf.setMatrix(this.genSecondOrderPos(this.leftCalfMatrix, this.leftCalfInitialMatrix,
        this.leftThighInitialMatrix));

    // Right calf Transformation
    this.rightCalfInitialMatrix = this.initialCalfMatrix();
    this.rightCalfMatrix = idMat4();
    this.rightCalfRotation = idMat4();
    this.rightCalfMatrix = this.transformMat(this.rightCalfMatrix, -this.calfRadius*this.calfLengthRescale,
        this.calfLengthRescale);
    this.rightCalf.setMatrix(this.genSecondOrderPos(this.rightCalfMatrix, this.rightCalfInitialMatrix,
        this.rightThighInitialMatrix));

	// Add robot to scene
	scene.add(this.torso);
    scene.add(this.head);
    scene.add(this.leftArm);
    scene.add(this.rightArm);
    scene.add(this.leftForearm);
    scene.add(this.rightForearm);
    scene.add(this.leftThigh);
    scene.add(this.rightThigh);
    scene.add(this.leftCalf);
    scene.add(this.rightCalf);
  }

  rotateTorso(angle){
    this.torsoMatrix = multMat(this.torsoMatrix, rotateMat(idMat4(), angle, "y"));
    var matrixTorso = multMat(this.torsoMatrix, this.torsoInitialMatrix);
    this.torso.setMatrix(matrixTorso);

    this.head.setMatrix(multMat(matrixTorso, multMat(this.headMatrix, this.headInitialMatrix)));

    this.leftArm.setMatrix(multMat(matrixTorso,multMat(this.leftArmInitialMatrix, this.leftArmMatrix)));

    this.leftForearm.setMatrix(multMat(matrixTorso, multMat(this.leftArmInitialMatrix,
        multMat(this.leftArmRotation, multMat(this.leftForearmInitialMatrix, this.leftForearmMatrix)))));

    this.rightArm.setMatrix(multMat(matrixTorso,multMat(this.rightArmInitialMatrix, this.rightArmMatrix)));

    this.rightForearm.setMatrix(multMat(matrixTorso, multMat(this.rightArmInitialMatrix,
        multMat(this.rightArmRotation, multMat(this.rightForearmInitialMatrix, this.rightForearmMatrix)))));

    this.leftThigh.setMatrix(multMat(matrixTorso,multMat(this.leftThighInitialMatrix, this.leftThighMatrix)));

    this.leftCalf.setMatrix(multMat(matrixTorso, multMat(this.leftThighInitialMatrix,
        multMat(this.leftThighRotation, multMat(this.leftCalfInitialMatrix, this.leftCalfMatrix)))));

    this.rightThigh.setMatrix(multMat(matrixTorso, multMat(this.rightThighInitialMatrix, this.rightThighMatrix)));

    this.rightCalf.setMatrix(multMat(matrixTorso, multMat(this.rightThighInitialMatrix,
        multMat(this.rightThighRotation, multMat(this.rightCalfInitialMatrix, this.rightCalfMatrix)))));
  }

  moveTorso(speed,forward){
    this.torsoMatrix = translateMat(this.torsoMatrix, speed * this.walkDirection.x, speed * this.walkDirection.y,
        speed * this.walkDirection.z);
    if(forward) {
      this.stickToGround();
    }
    var matrixTorso = multMat(this.torsoMatrix, this.torsoInitialMatrix);
    this.torso.setMatrix(multMat(this.torsoMatrix, this.torsoInitialMatrix));

    this.head.setMatrix(multMat(matrixTorso, multMat(this.headMatrix, this.headInitialMatrix)));

    this.leftArm.setMatrix(multMat(matrixTorso,multMat(this.leftArmInitialMatrix, this.leftArmMatrix)));

    this.leftForearm.setMatrix(multMat(matrixTorso, multMat(this.leftArmInitialMatrix,
        multMat(this.leftArmRotation, multMat(this.leftForearmInitialMatrix, this.leftForearmMatrix)))));

    this.rightArm.setMatrix(multMat(matrixTorso,multMat(this.rightArmInitialMatrix, this.rightArmMatrix)));

    this.rightForearm.setMatrix(multMat(matrixTorso, multMat(this.rightArmInitialMatrix,
        multMat(this.rightArmRotation, multMat(this.rightForearmInitialMatrix, this.rightForearmMatrix)))));

    if(forward) {
      if (this.walking === false) {
        this.resetLegForWalk();
      } else {
        this.walk();
      }
    } else {
      this.leftThigh.setMatrix(multMat(matrixTorso, multMat(this.leftThighInitialMatrix, this.leftThighMatrix)));

      this.leftCalf.setMatrix(multMat(matrixTorso, multMat(this.leftThighInitialMatrix,
          multMat(this.leftThighRotation, multMat(this.leftCalfInitialMatrix, this.leftCalfMatrix)))));

      this.rightThigh.setMatrix(multMat(matrixTorso, multMat(this.rightThighInitialMatrix, this.rightThighMatrix)));

      this.rightCalf.setMatrix(multMat(matrixTorso, multMat(this.rightThighInitialMatrix,
        multMat(this.rightThighRotation, multMat(this.rightCalfInitialMatrix, this.rightCalfMatrix)))));
    }
  }

  resetLegForWalk() {
    this.leftThighMatrix = multMat(inverseMat(this.leftThighRotation), this.leftThighMatrix );
    this.leftThighRotation = idMat4();
    this.leftThigh.setMatrix(multMat(multMat(this.torsoMatrix, this.torsoInitialMatrix),
        multMat(this.leftThighInitialMatrix, this.leftThighMatrix)));

    this.rightThighMatrix = multMat(inverseMat(this.rightThighRotation), this.rightThighMatrix);
    this.rightThighRotation = idMat4();
    this.rightThigh.setMatrix(multMat(multMat(this.torsoMatrix, this.torsoInitialMatrix),
        multMat(this.rightThighInitialMatrix, this.rightThighMatrix)));

    this.leftCalfMatrix = multMat(inverseMat(this.leftCalfRotation), this.leftCalfMatrix);
    this.leftCalfRotation = idMat4();
    this.leftCalf.setMatrix(multMat(multMat(this.torsoMatrix, this.torsoInitialMatrix),
        multMat(this.leftThighInitialMatrix, multMat(this.leftCalfInitialMatrix, this.leftCalfMatrix))));

    this.rightCalfMatrix = multMat(inverseMat(this.rightCalfRotation), this.rightCalfMatrix);
    this.rightCalfRotation = idMat4();
    this.rightCalf.setMatrix(multMat(multMat(this.torsoMatrix, this.torsoInitialMatrix),
        multMat(this.rightThighInitialMatrix, multMat(this.rightCalfInitialMatrix, this.rightCalfMatrix))));
  }

  stickToGround() {
    var legLenght = this.thighRadius*this.thighLengthRescale*2 + this.calfRadius*this.calfLengthRescale*2 + this.gap*2;
    if(0 < this.walkFrame && this.walkFrame < this.walkFrameCap/ 4  ||
        this.walkFrameCap/2 < this.walkFrame && this.walkFrame < this.walkFrameCap*3/4 ) {
      this.torsoMatrix = translateMat(this.torsoMatrix, 0,
          (Math.cos(this.bwdThighAngleVar*(this.walkFrame%(this.walkFrameCap/4 )))-
              Math.cos(this.bwdThighAngleVar*((this.walkFrame-1)%(this.walkFrameCap/4)))) * legLenght, 0);
    } else if (this.walkFrameCap/4 < this.walkFrame && this.walkFrame < this.walkFrameCap/2 ||
        this.walkFrameCap*3/4 < this.walkFrame && this.walkFrame < this.walkFrameCap) {
      this.torsoMatrix = translateMat(this.torsoMatrix, 0,
          (Math.cos(this.bwdThighAngleVar*(this.walkFrameCap/4 - this.walkFrame%(this.walkFrameCap/4)))-
              Math.cos(this.bwdThighAngleVar*(this.walkFrameCap/4 - (this.walkFrame-1)%(this.walkFrameCap/4)))) * legLenght, 0);
    } else if (this.walkFrame === this.walkFrameCap/4 || this.walkFrame === this.walkFrameCap*3/4) {
      this.torsoMatrix = translateMat(this.torsoMatrix, 0,
          (Math.cos(this.bwdThighAngleVar*(this.walkFrameCap/4 ))-
              Math.cos(this.bwdThighAngleVar*(this.walkFrameCap/4 -1))) * legLenght, 0);
    } 
  }

  walk() {
    if(this.walkFrame < this.walkFrameCap/4) {
      this.rotateLeftThigh(-this.fwdThighAngleVar);
      this.rotateRightThigh(this.bwdThighAngleVar);
      this.rotateLeftCalf(this.fwdCalfAngleVar);
      this.rotateRightCalf(-this.bwdCalfAngleVar);
    } else if (this.walkFrameCap/4 <= this.walkFrame && this.walkFrame < this.walkFrameCap/2) {
      this.rotateLeftThigh(this.fwdThighAngleVar);
      this.rotateRightThigh(-this.bwdThighAngleVar);
      this.rotateLeftCalf(-this.fwdCalfAngleVar);
      this.rotateRightCalf(this.bwdCalfAngleVar);
    } else if (this.walkFrameCap/2 <= this.walkFrame && this.walkFrame < this.walkFrameCap*3/4) {
      this.rotateRightThigh(-this.fwdThighAngleVar);
      this.rotateLeftThigh(this.bwdThighAngleVar);
      this.rotateRightCalf(this.fwdCalfAngleVar);
      this.rotateLeftCalf(-this.bwdCalfAngleVar);
    } else {
      this.rotateRightThigh(this.fwdThighAngleVar);
      this.rotateLeftThigh(-this.bwdThighAngleVar);
      this.rotateRightCalf(-this.fwdCalfAngleVar);
      this.rotateLeftCalf(this.bwdCalfAngleVar);
    }
    this.walkFrame++;
    if (this.walkFrame === this.walkFrameCap) {
      this.walkFrame = 0;
    }
  }

  rotateHead(angle){
    this.headMatrix = multMat(this.headMatrix, rotateMat(idMat4(), angle, "y"));
    this.head.setMatrix(multMat(this.torsoInitialMatrix, multMat(this.torsoMatrix,
        multMat(this.headMatrix, this.headInitialMatrix))));
  }

  rotateLeftArm(angle, axis){
    var rotation = rotateMat(idMat4(), angle, axis);
    this.leftArmRotation = multMat(rotation, this.leftArmRotation);
    this.leftArmMatrix = multMat(rotation, this.leftArmMatrix);

    var matrixLeftArm = multMat(this.torsoInitialMatrix, multMat(this.torsoMatrix, this.leftArmInitialMatrix));
    this.leftArm.setMatrix(multMat(matrixLeftArm,this.leftArmMatrix));
    this.leftForearm.setMatrix(multMat(matrixLeftArm,
        multMat(multMat(this.leftArmRotation,this.leftForearmInitialMatrix), this.leftForearmMatrix)));
  }

  rotateRightArm(angle, axis){
    var rotation = rotateMat(idMat4(), angle, axis);
    this.rightArmRotation = multMat(rotation, this.rightArmRotation);
    this.rightArmMatrix = multMat(rotation, this.rightArmMatrix);

    var matrixRightArm = multMat(this.torsoInitialMatrix, multMat(this.torsoMatrix, this.rightArmInitialMatrix));
    this.rightArm.setMatrix(multMat(matrixRightArm,this.rightArmMatrix));
    this.rightForearm.setMatrix(multMat(matrixRightArm,
        multMat(multMat(this.rightArmRotation,this.rightForearmInitialMatrix), this.rightForearmMatrix)));
  }

  rotateLeftForearm(angle){
    this.leftForearmMatrix = multMat(rotateMat(idMat4(), angle, "x"), this.leftForearmMatrix);
    this.leftForearm.setMatrix(multMat(this.torsoInitialMatrix, multMat(this.torsoMatrix,
        multMat(this.leftArmInitialMatrix, multMat(this.leftArmRotation,
            multMat(this.leftForearmInitialMatrix, this.leftForearmMatrix))))));
  }

  rotateRightForearm(angle){
    this.rightForearmMatrix = multMat(rotateMat(idMat4(), angle, "x"), this.rightForearmMatrix);
    this.rightForearm.setMatrix(multMat(this.torsoInitialMatrix, multMat(this.torsoMatrix,
        multMat(this.rightArmInitialMatrix, multMat(this.rightArmRotation,
            multMat(this.rightForearmInitialMatrix, this.rightForearmMatrix))))));
  }

  rotateLeftThigh(angle){
    var rotation = rotateMat(idMat4(),angle,"x");
    this.leftThighRotation = multMat(rotation, this.leftThighRotation);
    this.leftThighMatrix = multMat(rotation, this.leftThighMatrix);

    var matrixLeftThigh = multMat(this.torsoInitialMatrix, multMat(this.torsoMatrix, this.leftThighInitialMatrix));
    this.leftThigh.setMatrix(multMat(matrixLeftThigh,this.leftThighMatrix));
    this.leftCalf.setMatrix(multMat(matrixLeftThigh,
        multMat(multMat(this.leftThighRotation,this.leftCalfInitialMatrix), this.leftCalfMatrix)));
  }

  rotateRightThigh(angle){
    var rotation = rotateMat(idMat4(),angle,"x");
    this.rightThighRotation = multMat(rotation, this.rightThighRotation);
    this.rightThighMatrix = multMat(rotation, this.rightThighMatrix);

    var matrixRightThigh = multMat(this.torsoInitialMatrix, multMat(this.torsoMatrix, this.rightThighInitialMatrix));
    this.rightThigh.setMatrix(multMat(matrixRightThigh,this.rightThighMatrix));
    this.rightCalf.setMatrix(multMat(matrixRightThigh,
        multMat(multMat(this.rightThighRotation,this.rightCalfInitialMatrix), this.rightCalfMatrix)));
  }

  rotateLeftCalf(angle){
    var rotation = rotateMat(idMat4(), angle, "x");
    this.leftCalfRotation = multMat(rotation, this.leftCalfRotation);
    this.leftCalfMatrix = multMat(rotation, this.leftCalfMatrix);
    this.leftCalf.setMatrix(multMat(this.torsoInitialMatrix, multMat(this.torsoMatrix,
        multMat(this.leftThighInitialMatrix, multMat(this.leftThighRotation,
            multMat(this.leftCalfInitialMatrix, this.leftCalfMatrix))))));
  }

  rotateRightCalf(angle){
    var rotation = rotateMat(idMat4(), angle, "x");
    this.rightCalfRotation = multMat(rotation, this.rightCalfRotation);
    this.rightCalfMatrix = multMat(rotation, this.rightCalfMatrix);
    this.rightCalf.setMatrix(multMat(this.torsoInitialMatrix, multMat(this.torsoMatrix,
        multMat(this.rightThighInitialMatrix, multMat(this.rightThighRotation,
            multMat(this.rightCalfInitialMatrix, this.rightCalfMatrix))))));
  }
}

var robot = new Robot();

// LISTEN TO KEYBOARD
var keyboard = new THREEx.KeyboardState();

var selectedRobotComponent = 0;
var components = [
  "Torso",
  "Head",
  "Left arm",
  "Right arm",
  "Left forearm",
  "Right forearm",
  "Left thigh",
  "Right thigh",
  "Left calf",
  "Right calf"
];
var numberComponents = components.length;

function checkKeyboard() {
  // Next element
  if (keyboard.pressed("e")){
    selectedRobotComponent = selectedRobotComponent + 1;

    if (selectedRobotComponent<0){
      selectedRobotComponent = numberComponents - 1;
    }

    if (selectedRobotComponent >= numberComponents){
      selectedRobotComponent = 0;
    }

    window.alert(components[selectedRobotComponent] + " selected");
  }

  // Previous element
  if (keyboard.pressed("q")){
    selectedRobotComponent = selectedRobotComponent - 1;

    if (selectedRobotComponent < 0){
      selectedRobotComponent = numberComponents - 1;
    }

    if (selectedRobotComponent >= numberComponents){
      selectedRobotComponent = 0;
    }

    window.alert(components[selectedRobotComponent] + " selected");
  }

  // UP
  if (keyboard.pressed("w")){

    switch (components[selectedRobotComponent]){
      case "Torso":
        robot.moveTorso(0.1, true);
        break;
      case "Left arm":
        robot.rotateLeftArm(0.1, "z");
        break;
      case "Right arm":
        robot.rotateRightArm(-0.1, "z");
        break;
      case "Left forearm":
        robot.rotateLeftForearm(-0.1, "x");
        break;
      case "Right forearm":
        robot.rotateRightForearm(-0.1, "x");
        break;
      case "Left thigh":
        robot.rotateLeftThigh(-0.1);
        break;
      case "Right thigh":
        robot.rotateRightThigh(-0.1);
        break;
      case "Left calf":
        robot.rotateLeftCalf(-0.1);
        break;
      case "Right calf":
        robot.rotateRightCalf(-0.1);
        break;
      default :
        break;
    }
    if (components[selectedRobotComponent] === "Torso") {
      robot.walking = true;
    } else {
      robot.resetWalkingField();
    }
  }

  // DOWN
  if (keyboard.pressed("s")){
    switch (components[selectedRobotComponent]){
      case "Torso":
        robot.moveTorso(-0.1, false);
        break;
      case "Left arm":
        robot.rotateLeftArm(-0.1, "z");
        break;
      case "Right arm":
        robot.rotateRightArm(0.1, "z");
        break;
      case "Left forearm":
        robot.rotateLeftForearm(0.1, "x");
        break;
      case "Right forearm":
        robot.rotateRightForearm(0.1, "x");
        break;
      case "Left thigh":
        robot.rotateLeftThigh(0.1);
        break;
      case "Right thigh":
        robot.rotateRightThigh(0.1);
        break;
      case "Left calf":
        robot.rotateLeftCalf(0.1);
        break;
      case "Right calf":
        robot.rotateRightCalf(0.1);
        break;
      default :
        break;
    }
    if (components[selectedRobotComponent] === "Torso") {
      robot.walking = true;
    } else {
      robot.resetWalkingField();
    }
  }

  // LEFT
  if (keyboard.pressed("a")){
    switch (components[selectedRobotComponent]){
      case "Torso":
        robot.rotateTorso(0.1);
        break;
      case "Head":
        robot.rotateHead(0.1);
        break;
      case "Left arm":
        robot.rotateLeftArm(-0.1, "y");
        break;
      case "Right arm":
        robot.rotateRightArm(-0.1, "y");
        break;
      default :
        break;
    }
    if (components[selectedRobotComponent] === "Torso") {
      robot.walking = true;
    } else {
      robot.resetWalkingField();
    }
  }

  // RIGHT
  if (keyboard.pressed("d")){
    switch (components[selectedRobotComponent]){
      case "Torso":
        robot.rotateTorso(-0.1);
        break;
      case "Head":
        robot.rotateHead(-0.1);
        break;
      case "Left arm":
        robot.rotateLeftArm(0.1, "y");
        break;
      case "Right arm":
        robot.rotateRightArm(0.1, "y");
        break;
      default :
        break;
    }
    if (components[selectedRobotComponent] === "Torso") {
      robot.walking = true;
    } else {
      robot.resetWalkingField();
    }
  }

}

// SETUP UPDATE CALL-BACK
function update() {
  checkKeyboard();
  requestAnimationFrame(update);
  renderer.render(scene, camera);
}

update();