
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
#include <string>

// Here is a small helper for you! Have a look.
#include "grid.cpp"

#include "../../src/utils.cpp"
#include "Simulator.h"

#define WINDOW_SCALE 0.5f

const std::string RESOURCE_DIR = "Resources/";

int main(int, char const**)
{
    const unsigned int FRAMERATE = 60;
    
    sf::RenderWindow window(sf::VideoMode(1476 * WINDOW_SCALE, 1576 * WINDOW_SCALE), "Robot Simulator");
    window.setFramerateLimit(FRAMERATE);
    
    bool hasFocus = true;
    
//    sf::Image icon;
//
//    if (!icon.loadFromFile(resourcePath() + "HuskyRoboticsLogo.png")) {
//        return EXIT_FAILURE;
//    }
//    window.setIcon(icon.getSize().x, icon.getSize().y, icon.getPixelsPtr());
    
    Grid grid(40.f, 40.f, 36 * WINDOW_SCALE);
    Agent agent(grid.retrieveScale(), 2.f, 2.f);
    
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
                        grid.toggleGrid();
                        break;
                    }
                    case sf::Keyboard::O : {
                        grid.readObstaclesFromFile("../obstacles.txt");
                        std::cout << "Added obstacles" << std::endl;
                        break;
                    }
                    case sf::Keyboard::N : {
                        grid.toggleClipping();
                        break;
                    }
                    case sf::Keyboard::Num0 : {
                        agent.clearPath();
                        break;
                    }
                    case sf::Keyboard::Up : {
                        grid.moveAgent(agent, .5f);
                        break;
                    }
                    case sf::Keyboard::Down : {
                        grid.moveAgent(agent, -.5f);
                        break;
                    }
                    case sf::Keyboard::Left : {
                        grid.rotateAgent(agent, -15.f);
                        break;
                    }
                    case sf::Keyboard::Right : {
                        grid.rotateAgent(agent, 15.f);
                        break;
                    }
                }
            }
        }
        
        if (hasFocus) {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
                grid.moveAgent(agent, 10.f / FRAMERATE);
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
                grid.moveAgent(agent, -10.f / FRAMERATE);
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
                grid.rotateAgent(agent, -200.f / FRAMERATE);
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
                grid.rotateAgent(agent, 200.f / FRAMERATE);
            }
        }
        
        window.clear(sf::Color::White);
        window.draw(grid);
        window.draw(agent);
        window.display();
    }
    
    return EXIT_SUCCESS;
}
