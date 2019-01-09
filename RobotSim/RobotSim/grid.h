//
//  grid.hpp
//  RobotSim
//
//  Created by Tadeusz Pforte on 1/8/19.
//  Copyright © 2019 Husky Robotics. All rights reserved.
//

#include <stdio.h>
#include <SFML/Graphics.hpp>
#include "agent.h"

class Grid : public sf::Drawable, public sf::Transformable {
public:
    Grid (float w, float h, unsigned int s);
    
    void toggleGrid();
    void toggleClipping();
    
    void readObstaclesFromFile(std::string filename);
    void placeObstacle(float x1, float y1, float x2, float y2);
    
    sf::Vertex moveAgent(Agent &agent, float ds)
    float rotateAgent(Agent &agent, float dr)
    
    unsigned int retrieveScale();
    
    bool linesCollide(std::array<float, 4> line1, std::array<float, 4> line2);
    bool boxCollision(std::array<std::array<float, 4>, 4> box, std::array<float, 4> line);
    bool willCollide(Agent agent, float dx, float dy, float dr);
    
    const sf::Color BORDER_COLOR;
    const sf::Color GRID_COLOR;
    
    std::vector<Obstacle> obstacleList;
    
private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
    
    sf::VertexArray border;
    sf::VertexArray gridlines;
    
    bool showGrid;
    bool noclip;
    
    std::array<float, 4> TOP_BORDER;
    std::array<float, 4> RIGHT_BORDER;
    std::array<float, 4> BOTTOM_BORDER;
    std::array<float, 4> LEFT_BORDER;
    
    float width; // in meters
    float height; // in meters
    unsigned int scale; // pixels per meter
    
    void debugMsg(std::string msg);
};
