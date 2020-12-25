// Move
// -------------------------------------------------
function moveForward () {
  $.get('/move/forward', function( data ) {
    console.log(data)
  });
}

function moveBackward () {
  $.get('/move/backward', function( data ) {
    console.log(data)
  });

}

function moveStop () {
  $.get('/move/stop', function( data ) {
    console.log(data)
  });
}
// -------------------------------------------------

// Turn
// -------------------------------------------------
function turnLeft () {
  $.get('/turn/left', function( data ) {
    console.log(data)
  });
}

function turnRight () {
  $.get('/turn/right', function( data ) {
    console.log(data)
  });
}

function turnCenter () {
  $.get('/turn/center', function( data ) {
    console.log(data)
  });
}
// -------------------------------------------------

// Lights
// -------------------------------------------------
function lightsOn () {
  $.get('/lights/on', function( data ) {
    console.log(data)
  });
}

function lightsOff () {
  $.get('/lights/off', function( data ) {
    console.log(data)
  });
}
// -------------------------------------------------

// Lights
// -------------------------------------------------
function infraredOn () {
  $.get('/infrared/on', function( data ) {
    console.log(data)
  });
}

function infraredOff () {
  $.get('/infrared/off', function( data ) {
    console.log(data)
  });
}
// -------------------------------------------------

// Sensors
// -------------------------------------------------
function sensors () {
  $.get('/sensors', function( data ) {
    console.log(data)
  });
}
// -------------------------------------------------
