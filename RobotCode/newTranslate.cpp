#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <stack>
#include <unistd.h>
#include <Movement.h>
#include <Translate.h>


using namespace std;

int main( int argc, char* argv[] )
{
   ros::init(argc, argv, "listener");
   ros::NodeHandle nh;
   Translate decoder(nh);
   ros::spin();

   return 0;
}

Translate::Translate(ros::NodeHandle &nh) {
    _sub = nh.subscribe("code", 1, &Translate::setInput, this);
} 

Translate::~Translate() {}

void Translate::setInput(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    input = msg->data;
    if(input != "") {
        cout<<input<<endl;
        runProgram();
    }
}

// split the instruction string using commas as delimiter
vector<string> Translate::splitInstructions(){
    vector<string> result;
    stringstream ss (input);
    string item;
    while (getline (ss, item, ',')) {
        result.push_back (item);
    }
    return result;
}

// returns the index of the corresponding closing bracket
// takes the index of the opening bracket as a parameter
int Translate::indexLastBracket(int start, vector<char> firstLetters) {
    int bracketCloseCount = 1; // tracks closing/opening brackets
    while(bracketCloseCount != 0){
        start++;
        if(firstLetters[start] == '}' || firstLetters[start] == 'g'){
            bracketCloseCount--;
        } else if(firstLetters[start] == 'p' || firstLetters[start] == 'c' || firstLetters[start] == 'i' || firstLetters[start] == 'x'){
            bracketCloseCount++;
        }
    }
    return start;
}

// converts the ending brackets of loops to (goto start of loop)
void Translate::endBracketsToLoopGotos(vector<string> *instructions, vector<char> *firstLetters){
    for(int i = 0; i < instructions->size(); i++){
        if((*firstLetters)[i] == 'p' || (*firstLetters)[i] == 'c'){
            int lastBracket = indexLastBracket(i, *firstLetters);
            (*instructions)[lastBracket] = "g" + to_string(i);
            (*firstLetters)[lastBracket] = 'g';
        }
    }
}

void Translate::runProgram()
{
    Movement mover;
    // input = "w1,w2,w3,w10";//"m1,p2,f1,p7,f44,},i1,w3,},},k,f2";
    // //                    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
    vector<string> instructions = splitInstructions();
    vector<char> firstLetters;
    stack<int> loopReps;
    for(int i = 0; i < instructions.size(); i++){
        firstLetters.push_back(instructions[i].at(0));
    }
    endBracketsToLoopGotos(&instructions, &firstLetters);
    
    int index = 0;  // index of current instruction being processed
    double distance; 
    int unit; // 1 for feet, 0 for inches
    double degrees;
    bool pathBlocked = false;

    system("aplay media_startAudio.wav");
    while(index < instructions.size()){
        switch(firstLetters[index]){
            case 'f':
                distance = std::stod(instructions[index].substr(1, instructions[index].length() - 2));
                cout << "Moving forward " << distance;
                if(instructions[index].at(instructions[index].length() - 1) == 'f'){
                    // feet
                    unit = 1;
                    cout << " feet" << endl;
                } else {
                    // inches
                    unit = 0;
                    cout << " inches" << endl;
                }
                pathBlocked = mover.move(distance, unit);
                usleep(500000);
                break;
            case 'k':
                cout << "Moving forward until path blocked" << endl;
                break;
            case 'b':
                distance = std::stod(instructions[index].substr(1));
                cout << "Moving backward" << distance << " feet" << endl;
                pathBlocked = mover.move(-1 * distance, true);
                usleep(500000);
                break;
            case 'l':
                cout << "Turning left" << endl;
                mover.rotate(90);
                usleep(500000);
                break;
            case 'r':
                if(instructions[index].length() == 1){
                    // default right 90 degrees
                    cout << "Turning right" << endl;
                    mover.rotate(270);
                } else {
                    // specific degrees
                    degrees = std::stod(instructions[index].substr(1));
                    cout << "Turning " << degrees << " degrees" << endl;
                    mover.rotate(360 - degrees);
                }
                usleep(500000);
                break;
            case 'm':
                cout << "Moving to <insert location>" << endl;
                break;
            case 'p':{
                int reps = stoi(instructions[index].substr(1)); // original number of repeats
                loopReps.push(reps);
                instructions[index] = "x" + to_string(reps - 1);
                firstLetters[index] = 'x';
                }break;
            case 'x':{
                // repeat x times (during iterations)
                int curReps = stoi(instructions[index].substr(1));
                if(curReps == 0){
                    instructions[index] = "p" + to_string(loopReps.top());
                    firstLetters[index] = 'p';
                    loopReps.pop();
                    index = indexLastBracket(index, firstLetters); // will increment beyond index of last bracket later
                } else {
                    instructions[index] = "x" + to_string(curReps - 1);
                } 
                }break;
            case 'c':   // while loop is just an if statement with "goto start" at end
            case 'i':{
                int condition = stoi(instructions[index].substr(1));
                bool eval = false;
                if(condition == 1){
                    // if path blocked
                    eval = pathBlocked; // insert completed boolean here
                } else if(condition == 2){
                    eval = true; // insert completed boolean here
                } else if(condition == 3){
                    // if 50% chance occurs
                    int rVal = rand() % 100;
                    eval = rVal < 50;
                }

                if(!eval){
                    index = indexLastBracket(index, firstLetters); // if false, skip past block
                }
                }break;
            case 'w':
                cout << "Waiting " << instructions[index].substr(1) << " seconds" << endl;
                usleep(stoi(instructions[index].substr(1)) * 1000000);
                break;
            case 's':{
                int snd = stoi(instructions[index].substr(1));
                if(snd == 1){
                    system("aplay media_helloAudio.wav");
                } else if(snd == 2){
                    system("aplay media_byeAudio.wav");
                } else if(snd == 3){
                    system("aplay media_iamAudio.wav");
                } else if(snd == 4){
                    system("aplay media_niceDayAudio.wav");
                } else if(snd == 5){
                    system("aplay mixkit-happy-bell-alert-601.wav");
                } else if(snd == 6){
                    system("aplay mixkit-alert-quick-chime-766.wav");
                } else if(snd == 7){
                    system("aplay mixkit-sci-fi-error-alert-898.wav");
                }
            }break;
            case 'g':
                // goto instruction, jump to specific command
                // not part of official command set, } is converted to this
                index = stoi(instructions[index].substr(1)) - 1; // extract index
                break;
            case '}': break;
            default:
                cout << "There is an error with the input" << endl;
                break;
        }
        index++;
    }
    system("aplay media_doneAudio.wav");
}
