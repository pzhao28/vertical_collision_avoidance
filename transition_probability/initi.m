NATS_HOME = '/home/pzhao28/Documents/NATS';
NATS_Client = [NATS_HOME,'/','Client'];
cur_dir = pwd;
cd(NATS_Client);

javaaddpath([NATS_HOME, '/Client/dist/nats-client.jar'],'-end');
javaaddpath([NATS_HOME, '/Client/dist/nats-shared.jar'],'-end');

% server status constant
NATS_SIMULATION_STATUS_READY = com.osi.util.Constants.NATS_SIMULATION_STATUS_READY;
NATS_SIMULATION_STATUS_PAUSE = com.osi.util.Constants.NATS_SIMULATION_STATUS_PAUSE;
NATS_SIMULATION_STATUS_RESUME = com.osi.util.Constants.NATS_SIMULATION_STATUS_RESUME;
NATS_SIMULATION_STATUS_STOP = com.osi.util.Constants.NATS_SIMULATION_STATUS_STOP;
NATS_SIMULATION_STATUS_ENDED = com.osi.util.Constants.NATS_SIMULATION_STATUS_ENDED;

%connect server
natsClient = NATSClientFactory.getNATSClient();
simulationInterface = natsClient.getSimulationInterface();

environmentInterface = natsClient.getEnvironmentInterface();
equipmentInterface = natsClient.getEquipmentInterface();
aircraftInterface = equipmentInterface.getAircraftInterface();

entityInterface = natsClient.getEntityInterface();
safetyMetricsInterface = natsClient.getSafetyMetricsInterface();



simulationInterface.clear_trajectory();
