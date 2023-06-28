#include <vector>
#include <iostream>
#include <fstream>
#include <errno.h>
#include <cstring>
#include <string>
#include "environment.hpp"

#ifndef CSV_HPP
#define CSV_HPP

typedef std::vector<std::vector<std::vector<float> > > Data;


void CsvWrite(std::vector<Pylon> pylons, std::vector<Wall> walls, std::string l_file_name)
{
    std::fstream file_out;
    file_out.open(l_file_name, std::ios_base::out);

    if(!file_out.is_open())
    {
        std::cout << "failed to open";
    }
    file_out << "#Walls : p0.x; p0.y; v_dir.x; v_dir.y; d; door_pos; door_size\n";

    for (int i = 0; i < walls.size(); i++)
    {
        file_out << walls[i].p0.x << "; ";
        file_out << walls[i].p0.y << "; ";
        file_out << walls[i].v_dir.x << "; ";
        file_out << walls[i].v_dir.x << "; ";
        file_out << walls[i].d << "; ";
        file_out << walls[i].door_pos << "; ";
        file_out << walls[i].door_size << "; ";
        file_out << "\n";
    }

    file_out << "A\n\n";

    file_out << "#Pylons : pos.x; pos.y; radius\n";

    for (int i = 0; i < pylons.size(); i++)
    {
        file_out << pylons[i].pos.x << "; ";
        file_out << pylons[i].pos.y << "; ";
        file_out << pylons[i].radius << "; ";
        file_out << "\n";
    }

    file_out << "A\n";

}

std::vector<std::string> GetFields(const std::string& line)
{
    std::vector<std::string> fields;
    std::string curr_field = "";

    for (int i = 0; i < line.size(); i++)
    {
        char c = line[i];
        if (c == ';')
        {
            fields.push_back(curr_field);
            curr_field.clear();
        }
        else
        {
            curr_field.push_back(c);
        }

    }
    return fields;
}

Data CsvReader(const std::string& l_file_name)
{
    Data data;

    std::ifstream input_file;
    input_file.open(l_file_name.c_str());

    std::string line;

    if (!input_file)
    {
        std::cout << std::strerror(errno);
    }

    std::vector<std::vector<float> > curr_data;

    while(getline(input_file, line, '\n'))
    {
        if (line[0] == '#' || line.size() == 1){}
        else if(line[0] == 'A')
        {
            data.push_back(curr_data);
            curr_data.clear();
        }
        else
        {
            std::vector<std::string> fields = GetFields(line);
            std::vector<float> new_data;
            for (int k = 0; k < fields.size(); k++)
            {
                new_data.push_back(std::stof(fields[k]));
            }
            curr_data.push_back(new_data);
        }
    }

    input_file.close();

    return data;
}


#endif
