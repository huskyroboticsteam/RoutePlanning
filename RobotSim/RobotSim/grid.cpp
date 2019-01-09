//
//  map2.cpp
//  RobotSim
//
//  Created by Tadeusz Pforte on 12/4/18.
//  Copyright © 2018 Husky Robotics. All rights reserved.
//

#include <SFML/Graphics.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>

#include "obstacle.cpp"
#include "agent.cpp"

class Grid : public sf::Drawable, public sf::Transformable {
public:
    Grid (float w, float h, unsigned int s) {
        width = w;
        height = h;
        scale = s;
        
        TOP_BORDER = {0.f, 0.f, width - 1, 0.f};
        RIGHT_BORDER = {width - 1, 0.f, width - 1, height - 1};
        BOTTOM_BORDER = {0.f, height - 1, width - 1, height - 1};
        LEFT_BORDER = {0.f, 0.f, 0.f, height - 1};
        
        showGrid = false;
        noclip = false;
        
        border.setPrimitiveType(sf::LinesStrip);
        border.resize(5);
        
        border[0] = sf::Vector2f(scale, scale);
        border[1] = sf::Vector2f(scale, scale * width);
        border[2] = sf::Vector2f(scale * height, scale * width);
        border[3] = sf::Vector2f(scale * height, scale);
        border[4] = sf::Vector2f(scale, scale);
        
        border[0].color = BORDER_COLOR;
        border[1].color = BORDER_COLOR;
        border[2].color = BORDER_COLOR;
        border[3].color = BORDER_COLOR;
        border[4].color = BORDER_COLOR;
        
        gridlines.setPrimitiveType(sf::Lines);
        for (int x = 1; x < width; x++) {
            for (int y = 1; y < height; y++) {
                gridlines.append(sf::Vector2f(scale * x, scale * y));
                gridlines.append(sf::Vector2f(scale * x, scale * (y + 1)));
                gridlines.append(sf::Vector2f(scale * x, scale * y));
                gridlines.append(sf::Vector2f(scale * (x + 1), scale * y));
                gridlines[gridlines.getVertexCount() - 4].color = GRID_COLOR;
                gridlines[gridlines.getVertexCount() - 3].color = GRID_COLOR;
                gridlines[gridlines.getVertexCount() - 2].color = GRID_COLOR;
                gridlines[gridlines.getVertexCount() - 1].color = GRID_COLOR;
            }
        }
    }
    
    void toggleGrid() {
        showGrid = !showGrid;
    }
    
    void toggleClipping() {
        if (noclip)
            debugMsg("Clipping toggled on");
        else
            debugMsg("Clipping toggled off");
        
        noclip = !noclip;
    }
    
    void readObstaclesFromFile(std::string fileName) {
        std::ifstream file;
        file.open(fileName);
        if (file) {
            std::string line;
            
            while (getline(file, line)) {
                std::istringstream in(line);
                
                float x1 = 0.f, y1 = 0.f, x2 = 0.f, y2 = 0.f;
                
                in >> x1 >> y1 >> x2 >> y2;
                
                placeObstacle(x1, y1, x2, y2);
            }
        }
        file.close();
    }
    
    void placeObstacle(float x1, float y1, float x2, float y2) {
        obstacleList.push_back(Obstacle(x1, y1, x2, y2, scale, true));
    }
    
    sf::Vertex moveAgent(Agent &agent, float ds) {
        float curR = agent.getInternalRotation();
        float xOffset = ds * cos(curR * PI / 180);
        float yOffset = ds * sin(curR * PI / 180);
        
        if (!willCollide(agent, xOffset, yOffset, 0))
            agent.move(xOffset, yOffset);
    }
    
    float rotateAgent(Agent &agent, float dr) {
        if (!willCollide(agent, 0, 0, dr))
            agent.rotate(dr);
    }
    
    unsigned int retrieveScale() {
        return scale;
    }
    
    // returns true if the two lines, stored as {x1, y1, x2, y2}, intersect
    // otherwise returns false
    // sorry for variable names, but believe me it works 100%
    bool linesCollide(std::array<float, 4> line1, std::array<float, 4> line2) {
        float l1x1 = line1[0];
        float l1y1 = line1[1];
        float l1x2 = line1[2];
        float l1y2 = line1[3];
        
        float l2x1 = line2[0];
        float l2y1 = line2[1];
        float l2x2 = line2[2];
        float l2y2 = line2[3];
        
        float uA = ((l2x2-l2x1)*(l1y1-l2y1) - (l2y2-l2y1)*(l1x1-l2x1)) / ((l2y2-l2y1)*(l1x2-l1x1) - (l2x2-l2x1)*(l1y2-l1y1));
        
        float uB = ((l1x2-l1x1)*(l1y1-l2y1) - (l1y2-l1y1)*(l1x1-l2x1)) / ((l2y2-l2y1)*(l1x2-l1x1) - (l2x2-l2x1)*(l1y2-l1y1));
        
        return 0 <= uA && uA <= 1 && 0 <= uB && uB <= 1;
    }
    
    bool boxCollision(std::array<std::array<float, 4>, 4> box, std::array<float, 4> line) {
        bool flag = false;
        for (std::array<float, 4> boxLine : box) {
            if (linesCollide(boxLine, line)) {
                flag = true;
                break;
            }
        }
        return flag;
    }
    
    // returns true if the agent will collide with nothing in the transformation (dx, dy, dr)
    // returns false if there will be no collisions
    // is not very sophisticated (simply checks target location instead of path), TODO make better
    // currently only checks the map borders
    bool willCollide(Agent agent, float dx, float dy, float dr) {
        if (noclip)
            return false;
        
        int xQuadrant = agent.getX() / 4;
        int yQuadrant = agent.getY() / 4;
        
        std::array<std::pair<float, float>, 4> hitbox = agent.getHitBox();
        std::array<std::array<float, 4>, 4> hitboxLines;
        
        // apply rotation to hitbox
        for (int i = 0; i < 4; i++) {
            hitbox[i].second += (agent.getInternalRotation() + dr + 90) * PI / 180.f;
        }
        
        // convert to cartesian, and then create the lines
        for (int i = 0; i < 4; i++) {
            int end = i + 1;
            if (end > 3)
                end = 0;
            
            float r1 = hitbox[i].first;
            float t1 = hitbox[i].second;
            float r2 = hitbox[end].first;
            float t2 = hitbox[end].second;
            
            hitboxLines[i] = {r1 * cos(t1), r1 * sin(t1), r2 * cos(t2), r2 * sin(t2)};
            
            hitboxLines[i][0] += agent.getX() + dx;
            hitboxLines[i][1] += agent.getY() + dy;
            hitboxLines[i][2] += agent.getX() + dx;
            hitboxLines[i][3] += agent.getY() + dy;
        }
        
        bool flag = false;
        
        // check edge collisions
        if (xQuadrant == 0 && boxCollision(hitboxLines, LEFT_BORDER)) {
            flag = true;
            debugMsg("Hit left border");
        }
        else if (yQuadrant == 0 && boxCollision(hitboxLines, TOP_BORDER)) {
            flag = true;
            debugMsg("Hit top border");
        }
        else if (xQuadrant == (width / 4 - 1) && boxCollision(hitboxLines, RIGHT_BORDER)) {
            flag = true;
            debugMsg("Hit right border");
        }
        else if (yQuadrant == (height / 4 - 1) && boxCollision(hitboxLines, BOTTOM_BORDER)) {
            flag = true;
            debugMsg("Hit bottom border");
        }
        
        for (Obstacle o : obstacleList) {
            if (boxCollision(hitboxLines, {o.x1, o.y1, o.x2, o.y2})) {
                flag = true;
                debugMsg("Hit obstacle");
                break;
            }
        }
        
        return flag;
    }
    
    const sf::Color BORDER_COLOR = sf::Color::Black;
    const sf::Color GRID_COLOR = sf::Color(128, 128, 128);
    
private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {
        states.transform *= getTransform();
        
        if (showGrid)
            target.draw(gridlines, states);
        target.draw(border, states);
        for (Obstacle o : obstacleList) {
            target.draw(o, states);
        }
    }
    
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
    
    std::vector<Obstacle> obstacleList;
    
    void debugMsg(std::string msg) {
        std::cout << msg << std::endl;
    }
};
