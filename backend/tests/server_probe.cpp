#include "gvio/fusion.h"
#include <iostream>
int main(int argc,char** argv) {
    if (argc!=5) return 2;
    gvio::Config c; c.valid=true; c.filter.serverEnabled=true;
    c.filter.certPath=argv[1]; c.filter.keyPath=argv[2]; c.filter.serverPort=std::stoi(argv[3]); c.filter.webRoot=argv[4];
    c.filter.magEnabled=c.filter.gpsEnabled=false;
    gvio::FusionEngine engine(c);
    if (!engine.start()) return 3;
    std::cout<<"READY"<<std::endl;
    std::string command; std::getline(std::cin,command);
    engine.stop(); return 0;
}
