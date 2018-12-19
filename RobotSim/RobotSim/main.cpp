
//
// Disclaimer:
// ----------
//
// This code will work only if you selected window, graphics and audio.
//
// Note that the "Run Script" build phase will copy the required frameworks
// or dylibs to your application bundle so you can execute it on any OS X
// computer.
//
// Your resource files (images, sounds, fonts, ...) are also copied to your
// application bundle. To get the path to these resources, use the helper
// function `resourcePath()` from ResourcePath.hpp
//

#include <SFML/Audio.hpp>
#include <SFML/Graphics.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Here is a small helper for you! Have a look.
#include "ResourcePath.hpp"
#include "map.cpp"

int main(int, char const**)
{
    const unsigned int FRAMERATE = 60;
    
    sf::RenderWindow window(sf::VideoMode(1476, 1576), "Robot Simulator");
    window.setFramerateLimit(FRAMERATE);
    
    bool hasFocus = true;
    
    sf::Image icon;
    if (!icon.loadFromFile(resourcePath() + "HuskyRoboticsLogo.png")) {
        return EXIT_FAILURE;
    }
    window.setIcon(icon.getSize().x, icon.getSize().y, icon.getPixelsPtr());
    
    Map2 map(40.f, 40.f, 36);
    Agent2 agent(map.retrieveScale(), 2.f, 2.f);
    
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            // Close window on X or Cmd+W
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            else if (event.type == sf::Event::GainedFocus) {
                hasFocus = true;
            }
            else if (event.type == sf::Event::LostFocus) {
                hasFocus = false;
            }
            else if (event.type == sf::Event::KeyPressed && hasFocus) {
                switch (event.key.code) {
                    case sf::Keyboard::G : {
                        map.toggleGrid();
                        break;
                    }
                    case sf::Keyboard::O : {
                        map.readObstaclesFromFile("/Users/tadtiger/Documents/HuskyRobotics/RoutePlanning/RobotSim/RobotSim/obstacles.txt");
                        std::cout << "Added obstacles" << std::endl;
                        break;
                    }
                    case sf::Keyboard::N : {
                        map.toggleClipping();
                        break;
                    }
                    case sf::Keyboard::Num0 : {
                        agent.clearPath();
                        break;
                    }
                    case sf::Keyboard::Up : {
                        map.moveAgent(agent, .5f);
                        break;
                    }
                    case sf::Keyboard::Down : {
                        map.moveAgent(agent, -.5f);
                        break;
                    }
                    case sf::Keyboard::Left : {
                        map.rotateAgent(agent, -15.f);
                        break;
                    }
                    case sf::Keyboard::Right : {
                        map.rotateAgent(agent, 15.f);
                        break;
                    }
                }
            }
        }
        
        if (hasFocus) {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
                map.moveAgent(agent, 10.f / FRAMERATE);
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
                map.moveAgent(agent, -10.f / FRAMERATE);
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
                map.rotateAgent(agent, -200.f / FRAMERATE);
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
                map.rotateAgent(agent, 200.f / FRAMERATE);
            }
        }
        
        window.clear(sf::Color::White);
        window.draw(map);
        window.draw(agent);
        window.display();
    }
    
    return EXIT_SUCCESS;
}
