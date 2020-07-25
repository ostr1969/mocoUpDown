//#include <Moco/Components/DelpActuator.h>
#include <OpenSim/Actuators/DelpActuator.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <Moco/osimMoco.h>
#define MYFWD 1
#include "additions.h"
#include "console.h"
using namespace OpenSim;
using namespace SimTK;
using namespace std;
ofstream debugLog("results/fwd_debug.csv", ofstream::out);
ofstream Log1("results/fwd_1Table.csv", ofstream::out);
ofstream Log2("results/fwd_2Table.csv", ofstream::out);
ofstream Log3("results/fwd_3Table.csv", ofstream::out);
ofstream Log4("results/fwd_4Table.csv", ofstream::out);
ofstream Log5("results/fwd_5Table.csv", ofstream::out);
ofstream Log6("results/fwd_6Table.csv", ofstream::out);


class MyReporter : public PeriodicEventReporter {
public:
    MyReporter(const MultibodySystem& system, Real interval,PrescribedController* controller
                ,PathSpring& pathspring)
            : PeriodicEventReporter(interval), system(system),_controller(controller),
                _pathspring(pathspring) {}

    // Show x-y position of the pendulum weight as a function of time.
    void handleEvent(const State& state) const override {
        Vector_<SpatialVec>  forcesAtMInG;
        system.realize(state, Stage::Acceleration);
        system.getMatterSubsystem().calcMobilizerReactionForces( state,forcesAtMInG);
        SimTK::Vec3 comPos =
                system.getMatterSubsystem().calcSystemMassCenterLocationInGround(state);
        std::cout << state.getTime() << "\t," << forcesAtMInG[0][1][1] 
		<<"\t,"<< comPos(1)<< std::endl;



        Vector q=state.getQ();
        Vector u=state.getU();
        const Set<const Actuator>& fSet = _controller->getActuatorSet();
        const DelpActuator* act1=dynamic_cast<const DelpActuator*>(&fSet[0]);
        const DelpActuator* act2=dynamic_cast<const DelpActuator*>(&fSet[1]);
        const DelpActuator* act3=dynamic_cast<const DelpActuator*>(&fSet[2]);
        const DelpActuator* act4=dynamic_cast<const DelpActuator*>(&fSet[3]);
        const DelpActuator* act5=dynamic_cast<const DelpActuator*>(&fSet[4]);
        const DelpActuator* act6=dynamic_cast<const DelpActuator*>(&fSet[5]);
        Log1<< state.getTime()<<","<<q(1)<<","<<u(1)<<","<<act1->getControl(state)<<
        ","<<act1->getDelpVars(state,0)<<","<<act1->getDelpVars(state,1)<<","
        <<act1->getDelpVars(state,2)<<","<<act1->getDelpVars(state,3)
        <<","<<act1->get_coordinate()<<endl;
        Log2<< state.getTime()<<","<<q(2)<<","<<u(2)<<","<<act2->getControl(state)<<
        ","<<act2->getDelpVars(state,0)<<","<<act2->getDelpVars(state,1)<<","
        <<act2->getDelpVars(state,2)<<","<<act2->getDelpVars(state,3)
        <<","<<act2->get_coordinate()<<endl;
        Log3<< state.getTime()<<","<<q(3)<<","<<u(3)<<","<<act3->getControl(state)<<
        ","<<act3->getDelpVars(state,0)<<","<<act3->getDelpVars(state,1)<<","
        <<act3->getDelpVars(state,2)<<","<<act3->getDelpVars(state,3)
        <<","<<act3->get_coordinate()<<endl;
        Log4<< state.getTime()<<","<<q(1)<<","<<u(1)<<","<<act4->getControl(state)<<
        ","<<act4->getDelpVars(state,0)<<","<<act4->getDelpVars(state,1)<<","
        <<act4->getDelpVars(state,2)<<","<<act4->getDelpVars(state,3)
        <<","<<act4->get_coordinate()<<endl;
        Log5<< state.getTime()<<","<<q(2)<<","<<u(2)<<","<<act5->getControl(state)<<
        ","<<act5->getDelpVars(state,0)<<","<<act5->getDelpVars(state,1)<<","
        <<act5->getDelpVars(state,2)<<","<<act5->getDelpVars(state,3)
        <<","<<act5->get_coordinate()<<endl;
        Log6<< state.getTime()<<","<<q(3)<<","<<u(3)<<","<<act6->getControl(state)<<
        ","<<act6->getDelpVars(state,0)<<","<<act6->getDelpVars(state,1)<<","
        <<act6->getDelpVars(state,2)<<","<<act6->getDelpVars(state,3)
        <<","<<act6->get_coordinate()<<endl;

    }

private:
    const MultibodySystem& system;
    const PrescribedController* _controller;
    const PathSpring& _pathspring;

};
ofstream fwddebugLog("results/fwd_debug.csv", ofstream::out);


//
int main(int argc, char *argv[]){
	InpVars data=readvars();
    Log1<<"t,ang,vel,E,tou,act,fac,opt,coord\n";
    Log2<<"t,ang,vel,E,tou,act,fac,opt,coord\n";
    Log3<<"t,ang,vel,E,tou,act,fac,opt,coord\n";
    Log4<<"t,ang,vel,E,tou,act,fac,opt,coord\n";
    Log5<<"t,ang,vel,E,tou,act,fac,opt,coord\n";
    Log6<<"t,ang,vel,E,tou,act,fac,opt,coord\n";

	for (int i=0;i<2;i++) 
		cout<<data.ints[i].label<<":"<<data.ints[i].val<<endl;	
	for (int i=0;i<data.strings.size();i++) 
		cout<<data.strings[i].label<<":"<<data.strings[i].val<<endl;	
	for (int i=0;i<data.doubles.size();i++) 
		cout<<data.doubles[i].label<<":"<<data.doubles[i].val<<endl;	
	string modelFile="results/mycolo_initial.osim";
        //string controlsfile="mycolo_controls.sto";
        string statesFile="results/mycolo_states.bin";
	string controlsFile="results/mycolo_controls.bin";
        myTrajectory coloControls=binToTraj(controlsFile);
	cout<<"read conrols"<<endl;
        myTrajectory coloStates=binToTraj(statesFile);
	cout<<"read initial state"<<endl;
        Vector st=coloStates.getFirstRow();
	cout<<st<<endl;
        Model osimModel(modelFile);
        osimModel.updControllerSet().remove(0);
        osimModel.initSystem();
    OpenSim::Array<std::string> actuNames;
    const auto modelPath = osimModel.getAbsolutePath();
    for (const auto& actu : osimModel.getComponentList<DelpActuator>()) {
        actuNames.append(actu.getAbsolutePathString());
    }
        updateDelpActuator(osimModel, actuNames[0],"src/delp1.txt",.011,.068, 1,16);
        updateDelpActuator(osimModel, actuNames[1],"src/delp4.txt",.011,.068, 1,18);
        updateDelpActuator(osimModel, actuNames[2],"src/delp5.txt",.011,.068, 1,20);
        updateDelpActuator(osimModel, actuNames[3],"src/delp2.txt",.011,.068,-1,16);
        updateDelpActuator(osimModel, actuNames[4],"src/delp3.txt",.011,.068,-1,18);
        updateDelpActuator(osimModel, actuNames[5],"src/delp6.txt",.011,.068,-1,20);

    const SimTK::Vector& time = coloControls.time;


    auto* controller = new PrescribedController();
    controller->setName("prescribed_controller");

    for (int i = 0; i < actuNames.size(); ++i) {
        //const auto control = solution.getControl(actuNames[i]);
        const auto control = coloControls.getControl(actuNames[i]);
//        auto* fun = new GCVSpline(5, time.nrow(), &time[0], &control[0]);
        auto* funL=new PiecewiseLinearFunction(time.nrow(), &time[0], &control[0]);
//        auto* funC=new PiecewiseConstantFunction(time.nrow(), &time[0], &control[0]);

        const auto& actu = osimModel.getComponent<DelpActuator>(actuNames[i]);
        controller->addActuator(actu);
        controller->prescribeControlForActuator( actu.getName(), funL);
        fwddebugLog<<"time,"<<actuNames[i]<<endl;
        for (int j=0;j<time.nrow();j++)
        fwddebugLog<<time[j]<<","<<control[j]<<endl;
    }
    
    osimModel.addController(controller);
	
        State &si = osimModel.initSystem();
        auto& sp=osimModel.updComponent<PathSpring>("/forceset/path_spring");
	const MultibodySystem& system=osimModel.updMultibodySystem();
        system.addEventReporter(new MyReporter(system,.01,controller,sp));
        State &osimState = osimModel.initializeState();
	//set initial state from first line of statesfile
	cout<<"apply first line of states...."<<endl;
	//cout.precision(15);
	 for (int i=0; i<coloStates.numcols;i++)
	{osimModel.setStateVariableValue(osimState,coloStates.labels[i],st[i]);
         cout<<coloStates.labels[i]<<":"<<st[i]<<endl;}
	//cout<<"get end time from control file..."<<endl;
        double tf=time(time.nrow()-1);
	cout<<"Endtime:"<<tf<<endl;
	//system.realizeTopology();
        Manager manager(osimModel);
        si.getQ().dump("Initial q's");
        cout<<"read model from:"<< modelFile<<endl;
        cout<<"read traj solutionfrom:"<< controlsFile <<endl;
        cout<<"read initial state:"<< statesFile <<endl;
        osimModel.print("results/fwd_integ.osim");
        manager.initialize(osimState);

        std::cout<<"Integrating from 0  to " << tf << std::endl;
        manager.setIntegratorAccuracy(1.0e-4);
        cout<<"minStep:"<<tf/((time.nrow()-1))<<endl;
	//manager.setIntegratorMinimumStepSize(.015);
	manager.setIntegratorMaximumStepSize(tf/300);

        manager.integrate(tf);
        Vector_<SpatialVec>  forcesAtMInG;
        system.realize(manager.getState(), Stage::Acceleration);
        system.getMatterSubsystem().calcMobilizerReactionForces(manager.getState(),forcesAtMInG);
        SimTK::Vec3 comPos =
                system.getMatterSubsystem().calcSystemMassCenterLocationInGround(manager.getState());

        std::cout << manager.getState().getTime() << "\t," << forcesAtMInG[0][1][1] 
		<< "\t,"<<comPos[1]<<  std::endl;

        cout<<"end integration at time:"<<manager.getState().getTime()<<endl;
        osimModel.getMultibodySystem().realize(manager.getState(), Stage::Velocity);
        Vec3 COM_position = osimModel.getMultibodySystem().getMatterSubsystem().
                        calcSystemMassCenterLocationInGround(manager.getState());
        Vec3 COM_velocity = osimModel.getMultibodySystem().getMatterSubsystem().
                        calcSystemMassCenterVelocityInGround(manager.getState());
        double g = -osimModel.getGravity()[1];

        double maxHeight = (COM_velocity[1]>0?COM_position[1] +
                                pow(COM_velocity[1], 2.0)/(2.0*g):0);

        auto statesTable = manager.getStatesTable();
        STOFileAdapter::write(statesTable, "results/fwd_states.sto");
        cout<<"Wrote states to results/fwd_states.sto"<<endl;
	const Set<const Actuator>& actSet = osimModel.get_ControllerSet().get(0).getActuatorSet();

        //cout.precision(15);
	cout<<"pos:"<<COM_position<<endl;
        cout<<"vel:"<<COM_velocity<<endl;
	cout<<"jump:"<<maxHeight<<endl;
                

return 0;


}
