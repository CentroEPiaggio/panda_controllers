#include <iostream>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <iomanip>
#include <fstream>

#include <yaml-cpp/yaml.h>

#include "library/urdf2dh_inertial.h"

#define NUMLINKS 8

using namespace regrob;

using std::cout;
using std::endl;


int main(){

    std::vector<LinkProp> links_prop;

    //-------------------------------Extract Variables-----------------------------------//
    
    try {

        YAML::Node config = YAML::LoadFile("yaml/inertial.yaml");
        
        for (const auto& node : config) {

            LinkProp properties;
            
            std::string linkName = node.first.as<std::string>();
            YAML::Node mass = node.second["mass"];
            YAML::Node parI = node.second["inertia"];
            YAML::Node origin = node.second["origin"];
            std::string xyzStr, rpyStr;
            
            properties.name = linkName;
            
            properties.mass = mass.as<double>();
            properties.parI[0] = parI["xx"].as<double>();
            properties.parI[1] = parI["xy"].as<double>();
            properties.parI[2] = parI["xz"].as<double>();
            properties.parI[3] = parI["yy"].as<double>();
            properties.parI[4] = parI["yz"].as<double>();
            properties.parI[5] = parI["zz"].as<double>();
            
            if (origin["xyz"]) {
                
                xyzStr = origin["xyz"].as<std::string>();
                std::istringstream xyzStream(xyzStr);            
                xyzStream >> properties.xyz[0] >> properties.xyz[1] >> properties.xyz[2];
    
            } else {
                std::cerr << "Missing 'xyz' in the 'origin' node." << std::endl;
                continue;
            }

            if (origin["rpy"]) {
                rpyStr = origin["rpy"].as<std::string>();
                std::istringstream rpyStream(rpyStr);            
                rpyStream >> properties.rpy[0] >> properties.rpy[1] >> properties.rpy[2];
    
            } else {
                std::cerr << "Missing 'rpy' in the 'origin' node." << std::endl;
                continue;
            }

            links_prop.push_back(properties);
        }
        
    } catch (const YAML::Exception& e) {
        std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
    }

    /* for (const auto& link : links_prop) {
        std::cout << "link: " << link.name <<endl;
        std::cout << "Mass: " << link.mass << std::endl;
        std::cout << "Origin xyz: " << link.xyz << std::endl;
        std::cout << "Origin rpy: " << link.rpy << std::endl;
        std::cout << "Inertia: " << link.parI << std::endl;
        std::cout << std::endl;
    } */

    //-------------------------------Manipulate Variables-----------------------------------//
    
    std::vector<LinkProp> newlinks;
    std::vector<urdf2dh_T> transform;

    /* Define frame end-effctor respect last joint
    Obtained in $(find franka_description)/robots/common/franka_hand.xacro */
    //FrameOffset Ln_to_EE({-M_PI_4,0,0},{0,0,0.1034});
    
    transform.resize(NUMLINKS);
    transform[0].rpy = {+M_PI_2,0,0}; // link 1
    transform[1].rpy = {-M_PI_2,0,0};
    transform[2].rpy = {-M_PI_2,0,0};
    transform[3].rpy = {+M_PI_2,0,0};
    transform[4].rpy = {-M_PI_2,0,0};
    transform[5].rpy = {-M_PI_2,0,0};
    transform[6].rpy = {0,0,0+M_PI_4*0};

    //transform[6].rpy = {0,0,0};
    transform[7].rpy = {0,0,0}; // hand
    
    transform[0].xyz = {0,0,0}; // link 1
    transform[1].xyz = {0,0,0};
    transform[2].xyz = {-0.0825,0,0};
    transform[3].xyz = {0.0825,0,0};
    transform[4].xyz = {0,0,0};
    transform[5].xyz = {-0.088,0,0};
    transform[6].xyz = {0,0,-0.107-0.1034*0};
    //transform[6].xyz = {0,0,-0.107};
    transform[7].xyz = {0,0,0}; // hand

    newlinks.resize(NUMLINKS);

    for (int i = 1; i<=NUMLINKS; i++){
        trasformBodyInertial(transform[i-1].xyz,transform[i-1].rpy,links_prop[i],newlinks[i-1]);
    }

    /* Add hand to link 7 */

    LinkProp link_7, link_hand, newlink_7;

/*     link_7 = newlinks[6];
    link_hand = newlinks[7];
    
    newlink_7.name = link_7.name;
    newlink_7.mass = link_7.mass+link_hand.mass;

    for (int i=0; i<6;i++){
        newlink_7.parI[i] = link_7.parI[i]+link_hand.parI[i];
    }
    for (int i=0; i<3;i++){
        newlink_7.xyz[i] = link_7.xyz[i]+link_hand.xyz[i];
    } 
    newlinks[6] = newlink_7; */
    
    //-------------------------------Generate YAML-----------------------------------//
        
    YAML::Emitter emitter;
    emitter.SetIndent(2);
    emitter.SetSeqFormat(YAML::Flow);
    emitter << YAML::Comment(
        "Inertial parameters referred to joint's frame(!), only for arm + hand\n"
        "Format of link parameters [m, m*pgx, m*pgy, m*pgz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]\n"
        "Obtained with transformation of URDF file from $(find franka_gazebo)/test/launch/panda-gazebo.urdf\n"
        "and $(find franka_description)/robots/common/franka_robot.xacro\n"
        "or from $(find franka_description)/robots/common/inertial.yaml considering a specific Denavit-Hartenberg parametrization");
    emitter << YAML::Newline;

    try {
        
        for (int i=0; i<7; i++) {
            LinkProp link = newlinks[i];
            YAML::Node output;    
            YAML::Node linkNode;

            linkNode["mass"] = link.mass;

            linkNode["m_origin"]["xyz"] = link.xyz;

            linkNode["inertia"]["xx"] = link.parI[0];
            linkNode["inertia"]["xy"] = link.parI[1];
            linkNode["inertia"]["xz"] = link.parI[2];
            linkNode["inertia"]["yy"] = link.parI[3];
            linkNode["inertia"]["yz"] = link.parI[4];
            linkNode["inertia"]["zz"] = link.parI[5];

            std::string nodeName = link.name + "_params";

            emitter << YAML::BeginMap;
            emitter << YAML::Key << nodeName;
            emitter << YAML::Anchor(link.name);
            emitter << linkNode;
            emitter << YAML::EndMap << YAML::Newline;
        }

        YAML::Node control;
        std::string contName;
        contName = "default_controller";

        emitter << YAML::BeginMap;
        emitter << YAML::Key << contName << YAML::Anchor("default");
        emitter << YAML::BeginMap;            

        for (int i=0; i<7; i++) {
            LinkProp link = newlinks[i];
            emitter << YAML::Key << link.name << YAML::Value << YAML::Alias(link.name);
        } 
        
        emitter << YAML::EndMap << YAML::EndMap <<YAML::Newline;

        /* Backstepping controller */

        emitter << YAML::BeginMap;
        emitter << YAML::Key << "backstepping_controller" << YAML::Value << YAML::Alias("default");
        emitter << YAML::EndMap <<YAML::Newline;

        std::ofstream fout("../generatedFiles/inertial_DH.yaml");
        fout << emitter.c_str();
        fout.close();

        std::cout << "Successfully generated YAML file." << std::endl;

    } catch (const YAML::Exception& e) {
        std::cerr << "Error while generating YAML: " << e.what() << std::endl;
    }
    
    std::string absolutePath = std::filesystem::current_path();
    std::filesystem::path sourcePath = absolutePath + "/../generatedFiles/inertial_DH.yaml";
    std::filesystem::path sourceDestPath = "../../config/inertial_DH.yaml";
    std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::update_existing);

    return 0;
}