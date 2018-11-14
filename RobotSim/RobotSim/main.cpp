
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

// Here is a small helper for you! Have a look.
#include "ResourcePath.hpp"
#include "map.cpp"
#include "agent.cpp"

void processKeypresses(sf::Keyboard::Key key, Map &map, Agent &agent) {
    switch (key) {
        case sf::Keyboard::W :
            // go forward
            break;
        case sf::Keyboard::S :
            // go back
            break;
        case sf::Keyboard::A :
            // turn left
            break;
        case sf::Keyboard::D :
            // turn right
            break;
        case sf::Keyboard::O :
            map.placeObstacle(sf::Vector2f(240.f, 240.f));
            break;
        default:
            std::cout << "Invalid input" << std::endl;
            break;
    }
}

int main(int, char const**)
{
    sf::RenderWindow window(sf::VideoMode(1024, 1024), "Robot Simulator");

    sf::Image icon;
    if (!icon.loadFromFile(resourcePath() + "HuskyRoboticsLogo.png")) {
        return EXIT_FAILURE;
    }
    window.setIcon(icon.getSize().x, icon.getSize().y, icon.getPixelsPtr());

    Map map(40.f, 40.f, 24);
    
    Agent agent(map.getOrigin());
    
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
                processKeypresses(event.key.code, map, agent);
            }
        }

        
        window.clear(sf::Color::White);
        window.draw(map);
        window.draw(agent);
        window.display();
    }

    return EXIT_SUCCESS;
}
