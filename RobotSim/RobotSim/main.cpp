
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
#include "agent.cpp"

int main(int, char const**)
{
    sf::RenderWindow window(sf::VideoMode(1024, 1024), "Robot Simulator");

    sf::Image icon;
    if (!icon.loadFromFile(resourcePath() + "HuskyRoboticsLogo.png")) {
        return EXIT_FAILURE;
    }
    window.setIcon(icon.getSize().x, icon.getSize().y, icon.getPixelsPtr());

    Map map(40.f, 40.f, 24);
    
    Agent agent(map.getOrigin(), map);
    
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            // Close window on X or Cmd+W
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            
            else if (event.type == sf::Event::KeyPressed) {
                switch (event.key.code) {
                    case sf::Keyboard::P : {
                        std::cout <<
                        "Internal position: (" + std::to_string(agent.getX()) + "," +
                        std::to_string(agent.getY()) + ") and " + std::to_string(agent.getRotation()) + " degrees"
                        << std::endl;
                        break;
                    }
                    case sf::Keyboard::I : {
                        map.readObstaclesFromFile("/Users/tadtiger/Documents/HuskyRobotics/RoutePlanning/RobotSim/RobotSim/obstacles.txt");
                        break;
                    }
                    default:
                        // do nothing
                        break;
                }
            }
            
        }
        
        
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
            agent.move(0.016f);
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
            agent.move(-0.016f);
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
            agent.rotate(-0.5f);
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
            agent.rotate(0.5f);
        }

        
        window.clear(sf::Color::White);
        window.draw(map);
        window.draw(agent);
        window.display();
    }

    return EXIT_SUCCESS;
}
