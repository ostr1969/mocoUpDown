import opensim as osim
from math import pi
from opensim import Vec3
from opensim import Constant
from pyfun import addActivationCoordinateActuator
osimModel=osim.Model()
osimModel.setAuthors("barak")
osimModel.setName("baraksJump")
ground=osimModel.updGround()
ground.attachGeometry(osim.Mesh("checkered_floor.vtp"))
linkageMass1 = 2.32 ; linkageLength1 = 0.2084; linkageDiameter = 0.01
linkageMass2 = 7.44 ; linkageLength2 = 0.4182
linkageMass3 = 16   ; linkageLength3 = 0.4165
linkageMass4 = 54.24; linkageLength4 = 0.7780
linkageMassCenter1=osim.Vec3(0.,linkageLength1-0.1140836,0.)
linkageMassCenter2=osim.Vec3(0.,linkageLength2-0.18647538,0.)
linkageMassCenter3=osim.Vec3(0.,linkageLength3-0.17055675,0.)
linkageMassCenter4=osim.Vec3(0.,linkageLength4-0.4237,0.)
exothick=0.025
exocenter=osim.Vec3(0.,exothick/2,0.)
linkage1 =osim.Body("foot", linkageMass1,linkageMassCenter1,
				 osim.Inertia(0.1,0.1,0.00662,0.,0.,0.))
linkage2 = osim.Body("shank", linkageMass2,
                linkageMassCenter2, osim.Inertia(0.1,0.1,0.1057,0.,0.,0.));
linkage3 = osim.Body("thigh", linkageMass3,
                linkageMassCenter3, osim.Inertia(0.2,0.1,0.217584,0.,0.,0.));
linkage4 = osim.Body("HAT", linkageMass4,
                linkageMassCenter4, osim.Inertia(1.1,1.1,1.48,0.,0.,0.));
ZAxis=osim.CoordinateAxis(2)
R=osim.Rotation(-pi/2, ZAxis)
sphere=osim.Sphere(0.04)
exo=osim.Cylinder(.05, exothick)
linkage1.attachGeometry(sphere.clone());
cyl1=osim.Cylinder(linkageDiameter/2, linkageLength1/2)
cyl1Frame = osim.PhysicalOffsetFrame(linkage1,
            osim.Transform(osim.Vec3(0.0, linkageLength1/2 , 0.0)))
cyl1Frame.setName("Cyl1_frame");
cyl1Frame.attachGeometry(cyl1.clone());
osimModel.addComponent(cyl1Frame)

linkage2.attachGeometry(sphere.clone());
cyl2=osim.Cylinder(linkageDiameter/2, linkageLength2/2);
cyl2Frame = osim.PhysicalOffsetFrame(linkage2,
            osim.Transform(osim.Vec3(0.0, linkageLength2/2 , 0.0)));
cyl2Frame.setName("Cyl2_frame");
cyl2Frame.attachGeometry(cyl2.clone());
cyl2Frame.attachGeometry(exo.clone());
osimModel.addComponent(cyl2Frame);

linkage3.attachGeometry(sphere.clone());
cyl3=osim.Cylinder(linkageDiameter/2, linkageLength3/2);
cyl3Frame = osim.PhysicalOffsetFrame(linkage3,
            osim.Transform(osim.Vec3(0.0, linkageLength3/2 , 0.0)));
cyl3Frame.setName("Cyl3_frame");
cyl3Frame.attachGeometry(cyl3.clone());
cyl3Frame.attachGeometry(exo.clone());
osimModel.addComponent(cyl3Frame);

linkage4.attachGeometry(sphere.clone());
cyl4=osim.Cylinder(linkageDiameter/2, linkageLength4/2);
cyl4Frame = osim.PhysicalOffsetFrame(linkage4,
            osim.Transform(osim.Vec3(0.0, linkageLength4 / 2.0, 0.0)));
cyl4Frame.setName("Cyl4_frame");
cyl4Frame.attachGeometry(cyl4.clone());
osimModel.addComponent(cyl4Frame);

orientationInGround=Vec3(0.);
locationInGround=Vec3(0.);
locationInParent1=Vec3(0.0, linkageLength1, 0.0);
locationInParent2=Vec3(0.0, linkageLength2, 0.0);
locationInParent3=Vec3(0.0, linkageLength3, 0.0);
locationInParent4=Vec3(0.0, linkageLength4, 0.0);
orientationInChild=Vec3(0.);
locationInChild=Vec3(0.);

tip   = osim.PinJoint("tip",
                ground, locationInGround, orientationInGround,
                linkage1, locationInChild, orientationInChild);

ankle = osim.PinJoint("ankle",
                linkage1, locationInParent1, orientationInChild,
                linkage2, locationInChild, orientationInChild);

knee = osim.PinJoint("knee",
                linkage2, locationInParent2, orientationInChild,
                linkage3, locationInChild, orientationInChild);

hip = osim.PinJoint("hip",
                linkage3, locationInParent3, orientationInChild,
                linkage4, locationInChild, orientationInChild);

toeRange = [0*pi/180, pi/2];
ankleRange = [10*pi/180-pi,20*pi/180-pi];
kneeRange = [0, pi-30*pi/180];
hipRange = [50*pi/180-pi,60*pi/180-pi];

tip.updCoordinate().setName("q0");
tip.updCoordinate().setRangeMin(toeRange[0])
tip.updCoordinate().setRangeMax(toeRange[1]);
ankle.updCoordinate().setName("q1");
ankle.updCoordinate().setRangeMin(ankleRange[0])
ankle.updCoordinate().setRangeMax(ankleRange[1]);
knee.updCoordinate().setName("q2");
knee.updCoordinate().setRangeMin(kneeRange[0])
knee.updCoordinate().setRangeMax(kneeRange[1]);
hip.updCoordinate().setName("q3");
hip.updCoordinate().setRangeMin(hipRange[0])
hip.updCoordinate().setRangeMax(hipRange[1]);

allStiff = 10000; allDamping = 5.; allTransition = 10.;
toeLimitForce = osim.CoordinateLimitForce("q0", toeRange[1]*180/pi,
        allStiff, toeRange[0]*180/pi, allStiff, allDamping, allTransition);
ankleLimitForce = osim.CoordinateLimitForce("q1", ankleRange[1]*180/pi,
        allStiff, ankleRange[0]*180/pi, allStiff, allDamping, allTransition);
kneeLimitForce = osim.CoordinateLimitForce("q2", kneeRange[1]*180/pi,
        allStiff, kneeRange[0]*180/pi, allStiff, allDamping, allTransition);
hipLimitForce = osim.CoordinateLimitForce("q3", hipRange[1]*180/pi,
        allStiff, hipRange[0]*180/pi, allStiff, allDamping, allTransition);

osimModel.addBody(linkage1);
osimModel.addBody(linkage2);
osimModel.addBody(linkage3);
osimModel.addBody(linkage4);

osimModel.addJoint(tip);
osimModel.addJoint(ankle);
osimModel.addJoint(knee);
osimModel.addJoint(hip);

a1=addActivationCoordinateActuator(osimModel, "q1","ap");
a2=addActivationCoordinateActuator(osimModel, "q2","kp");
a3=addActivationCoordinateActuator(osimModel, "q3","hp");
a_1=addActivationCoordinateActuator(osimModel, "q1","am");
a_2=addActivationCoordinateActuator(osimModel, "q2","km");
a_3=addActivationCoordinateActuator(osimModel, "q3","hm");

actcontroller =osim.PrescribedController();
actcontroller.addActuator(a1);
actcontroller.addActuator(a2);
actcontroller.addActuator(a3);
actcontroller.addActuator(a_1);
actcontroller.addActuator(a_2);
actcontroller.addActuator(a_3);

actcontroller.prescribeControlForActuator("ap", Constant(0));
actcontroller.prescribeControlForActuator("kp", Constant(0));
actcontroller.prescribeControlForActuator("hp", Constant(0));
actcontroller.prescribeControlForActuator("am", Constant(0));
actcontroller.prescribeControlForActuator("km", Constant(0));
actcontroller.prescribeControlForActuator("hm", Constant(0));
actcontroller.set_interpolation_method(3);
osimModel.addController(actcontroller);
osimModel.setGravity(Vec3(0., -9.81   , 0.));

pullymass=.001;resting_length=0.4;stiffness=4454.76*4.;dissipation=0.01
pulleyBody = osim.Body("PulleyBody", pullymass ,Vec3(0),  osim.Inertia(0.1, 0.1, 0.1,0.1,0.1,0.1)); 
#brick.inertia??
pulley = osim.WrapCylinder();
pulley.set_radius(0.05);
pulley.set_length(0.05);
pulley.set_quadrant("-y");

pulleyBody.addWrapObject(pulley);
osimModel.addBody(pulleyBody);
weld = osim.WeldJoint("weld", linkage3, Vec3(0), Vec3(0), pulleyBody, Vec3(0), Vec3(0));
osimModel.addJoint(weld);

spring =osim.PathSpring("path_spring",resting_length,stiffness ,dissipation);
spring.updGeometryPath().appendNewPathPoint("origin", linkage3, Vec3(0.05, 0.2, 0));
spring.updGeometryPath().appendNewPathPoint("insert", linkage2, Vec3(0.05,linkageLength2-.2,0));
spring.updGeometryPath().addPathWrap(pulley);

osimModel.addForce(spring);
osimModel.finalizeConnections();

osimModel.buildSystem();

si = osimModel.initializeState()
coordinates = osimModel.updCoordinateSet();
coordinates.get(0).setValue(si, 0, True);
coordinates.get(1).setValue(si,1, True);
coordinates.get(2).setValue(si, 2, True);
coordinates.get(3).setValue(si,3, True);




