syms t real

Rotx = [1 0 0; 0 cos(t) -sin(t); 0 sin(t) cos(t)];
Roty = [cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)];
Rotz = [cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];

Commentx = "This is a function that calculates the 3x3 rotation matrix around the x-axis for input t in radians.";
Commenty = "This is a function that calculates the 3x3 rotation matrix around the y-axis for input t in radians.";
Commentz = "This is a function that calculates the 3x3 rotation matrix around the z-axis for input t in radians.";

matlabFunction(Rotx, 'File', 'Rotx.m', 'Optimize', true, 'Comments', Commentx, 'Vars', 't');
matlabFunction(Roty, 'File', 'Roty.m', 'Optimize', true, 'Comments', Commenty, 'Vars', 't');
matlabFunction(Rotz, 'File', 'Rotz.m', 'Optimize', true, 'Comments', Commentz, 'Vars', 't');

clear Rotx Roty Rotz t Commentx Commenty Commentz