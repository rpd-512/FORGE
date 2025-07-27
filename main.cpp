#include "src/src.h"
#include "metaheuristics/socialGroupOptimization/main.cpp"
#include "metaheuristics/geneticAlgorithm/main.cpp"
#include "metaheuristics/particleSwarmOptimization/main.cpp"
#include "metaheuristics/teachingLearningBasedOptimization/main.cpp"
#include "metaheuristics/differentialEvolution/main.cpp"
#include "gradientDescent/main.cpp"

void clear_screen() {
    #ifdef _WIN32
        system("cls");
    #else
        system("clear");
    #endif
}

int count_lines(const string& file_yaml) {
    ifstream file(file_yaml);
    if (!file.is_open()) {
        return 0; // File doesn't exist or cannot be opened
    }

    int count = 0;
    string line;
    while (getline(file, line)) {
        ++count;
    }
    return count;
}

atomic<int> dataset_size(0);
mutex mtx;  // Mutex for protecting shared resource access

void gen_set(int pop, int itr, RobotInfo robot, CSVWriter& writer){
    int dim = robot.dof;
    robot.joint_angle = {};
    for (int i = 0; i < dim; ++i) {
        robot.joint_angle.push_back(0);
        //robot.joint_angle.push_back(uniform(0, 2*M_PI));
    }

    float net_radius = 0;
    for (dh_param dh : robot.dh_params) {
        net_radius += sqrt(dh.a * dh.a + dh.d * dh.d);
    }

    float theta = uniform(0, 2 * M_PI);           // azimuth angle
    float phi = acos(uniform(0, 1));              // zenith angle, limits Z ≥ 0
    float r = cbrt(uniform(0, 1)) * net_radius;   // cube root for uniform radial distribution

    robot.destination.x = r * sin(phi) * cos(theta);
    robot.destination.y = r * sin(phi) * sin(theta);
    robot.destination.z = r * cos(phi);           // ensures z ≥ 0

    robot.init_pos = forward_kinematics(robot.joint_angle,robot).back();


    vector<vector <float>> randGen = generateChromosome(pop,dim);

    plotPoint optima, post_gd;
    optima.fitness = numeric_limits<double>::max();

    plotPoint pso = particleSwarmOptimization(pop,itr,randGen,robot);
    if(optima.fitness > pso.fitness) optima = pso;
    plotPoint ga = geneticAlgorithm(pop,itr,randGen,robot);
    if(optima.fitness > ga.fitness) optima = ga;
    plotPoint sgo = socialGroupOptimization(pop,itr,randGen,robot);
    if(optima.fitness > sgo.fitness) optima = sgo;
    plotPoint tlbo = teachingLearningBasedOptimization(pop,itr,randGen,robot);
    if(optima.fitness > tlbo.fitness) optima = tlbo;
    plotPoint de = differentialEvolutionAlgorithm(pop,itr,randGen,robot);
    if(optima.fitness > de.fitness) optima = de;


    position3D dist = forward_kinematics(optima.best_gene, robot).back();
    optima.fitness = distance(dist, robot.destination);
    float alpha = 0.001;  // learning rate
    post_gd = gradientDescent(20000, alpha, optima.best_gene, robot);
    if(post_gd.fitness < optima.fitness){
        post_gd.name = optima.name+"_"+post_gd.name;
        optima = post_gd;
    }
    if(optima.fitness > 1 || SceneCollisionCheck(robot.scene_objects, forward_kinematics(optima.best_gene,robot))){return;}

    optima.best_gene = normalize_angle(optima.best_gene);

    vector<float> inputLayer = robot.joint_angle;
    vector<float> posVector = {robot.destination.x,robot.destination.y,robot.destination.z};
    inputLayer.insert(inputLayer.end(),posVector.begin(),posVector.end());
    vector<float> outputLayer = optima.best_gene;
    string misc = optima.name; 
    clear_screen();
    cout << " _____ ___  ____   ____ _____ \n"; 
    cout << "|  ___/ _ \\|  _ \\ / ___| ____|\n"; 
    cout << "| |_ | | | | |_) | |  _|  _|  \n"; 
    cout << "|  _|| |_| |  _ <| |_| | |___ \n"; 
    cout << "|_|   \\___/|_| \\_\\\\____|_____|\n"; 
    cout << "Formation of Optimized Robotic Groundtruth Examples\n\n";

    cout << "Thankyou for your contribution" << endl;
    cout << "Simulated Robot: " << robot.name << endl;
    cout << "You have successfully added " << ++dataset_size
          << " datapoint(s) to the dataset in \"" << robot.name << ".csv\"." << endl;

    lock_guard<mutex> lock(mtx);
    writer.appendData(inputLayer, outputLayer, misc);
}

void thread_worker(int pop, int itr, RobotInfo robot, CSVWriter& writer){
    while(1){
        gen_set(pop, itr, robot, writer);
    }
}


int main(int argc, char* argv[]){
    if (argc < 3) {
        cerr << "Usage: " << argv[0] << " <config.yaml> <num_cores>\n"
          << "  <config.yaml> : Path to the YAML file containing DH parameters\n"
          << "  <scene.json>  : Path to the JSON file containing Scene Data (Use scene_designer/Designer.py to create scene)\n"
          << "  <num_cores>   : Number of CPU cores to assign to the task\n";
        return 1;
    }

    string file_yaml = argv[1];
    cout << "Using YAML file: " << file_yaml << std::endl;
    string file_json = argv[2];
    cout << "Using JSON file: " << file_json << std::endl;
    RobotInfo robot;

    srand(time(0));
    

    robot.name = get_robot_name(file_yaml);
    // a, d, alpha
    if (loadDHFromYAML(file_yaml, robot)) {
        cout << "Loaded DH parameters:\n";
        for (size_t i = 0; i < robot.dh_params.size(); ++i) {
            auto& dh = robot.dh_params[i];
            cout << "Joint " << i+1 << ": a=" << dh.a << ", d=" << dh.d << ", alpha=" << dh.alpha << endl;
        }
    }
    else {cerr << "Failed to load DH parameters.\n";}

    loadSceneFromJSON(file_json, robot);
    if (robot.scene_objects.empty()) {
        cerr << "No scene objects loaded from JSON file." << endl;
        return 1;
    } else {
        cout << "Loaded " << robot.scene_objects.size() << " scene objects from JSON file." << endl;
    }   

    robot.dof = robot.dh_params.size();

    CSVWriter writer(robot);
    dataset_size = count_lines(robot.name+".csv")-1;
    int pop = 100;
    int itr = 100;


    vector<thread> threads;
    unsigned int cores = thread::hardware_concurrency();
    // Launch threads
    for (int i = 0; i < stoi(argv[3]); ++i) {
        threads.emplace_back(thread_worker, ref(pop), itr, ref(robot), ref(writer));
    }

    // Join threads (main thread waits indefinitely)
    for (auto& t : threads) {
        t.join();
    }

    return 0;
}