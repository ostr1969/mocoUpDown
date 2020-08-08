import opensim as osim
def addActivationCoordinateActuator(model,coordName, actuname,  act=0): 

    coordSet = model.updCoordinateSet();

    actu = osim.ActivationCoordinateActuator();
    actu.set_activation_time_constant(0.011);
#    actu.set_dactivation_time_constant(0.068);
    actu.set_default_activation(act);
    actu.setName(actuname);
    actu.setCoordinate(coordSet.get(coordName));
    actu.setOptimalForce(1);
    actu.setMinControl(-1);
    actu.setMaxControl(1);
    model.addComponent(actu);
    return actu;


