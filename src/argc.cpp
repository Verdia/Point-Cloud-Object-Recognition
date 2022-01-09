#include <iostream>

std::string getExtension(std::string inputString){
    return inputString.substr(inputString.find_last_of("."), inputString.length());
}

std::string getName(std::string inputString){
    return inputString.substr(0, inputString.find_last_of("."));
}

int main(int argc, char *argv[]){
    std::string fileName(getName(argv[1]));
    std::string fileName2(getName(argv[2]));
    std::string fileName3(getName(argv[3]));
    std::string fileName4(getName(argv[4]));
    std::string fileName5(getName(argv[5]));
    std::string fileExtension(getExtension(argv[1]));
    
    std::cerr << "nama file: " << fileName << std::endl;
    std::cerr << "nama file: " << fileName2 << std::endl;
    std::cerr << "nama file: " << fileName3 << std::endl;
    std::cerr << "nama file: " << fileName4 << std::endl;
    std::cerr << "nama file: " << fileName5 << std::endl;
    std::cerr << "nama file: " << fileExtension << std::endl;

    return 0;
}