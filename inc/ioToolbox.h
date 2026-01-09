/*Functions for reading target and control parameters from files and for printing metadata and simulation output.
* Used by simulate_reaches_rate.cpp
*/

#pragma once

#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <ctime>
#include <cstdlib>

#include <time.h> 
#include <string>
#include <vector>

#include <iostream>
#include <iomanip>
#include <fstream>


void importTargets(std::ifstream& targetfile, std::vector<std::pair<double, double>>& targ_list, mjControl& c)
{
    std::string line;
    std::string::size_type sz;
    std::string::size_type st;

    int numberOfLines = 0;
    int varInd = 0;
    double x_targ[2];
    while (std::getline(targetfile, line))
    {
        st = 0;
        for (int varInd = 0; varInd < 2; varInd++)
        {
            x_targ[varInd] = std::stod(line.substr(st), &sz);
            sz++; // correction due to separator ";\t"
            st += sz;
        }
        targ_list.push_back(std::make_pair(x_targ[0], x_targ[1]));
        ++numberOfLines;
    }

    std::cout << "Reading targets completed with " << numberOfLines << " lines of data.\n";
    c.ntargs = numberOfLines;
}

void importScaledControlParameters(std::ifstream& ctrlfile, mjControl& c)
{
    std::string line;
    std::string::size_type sz;
    std::string::size_type st;

    int dofs = 3;
    int varInd = 0;
    double temp;
    
    for (int lineNumber = 0; lineNumber < dofs; lineNumber++)
    {
        std::getline(ctrlfile, line);
        
        st = 0;
        for (int varInd = 0; varInd < 2; varInd++)
        {
            temp = std::stod(line.substr(st), &sz);
            if (varInd == 0)
                temp = temp * 100.0;
            if (varInd == 1)
                temp = sqrt(c.K[dofs * lineNumber]) * temp;
            c.K[dofs * lineNumber + varInd] = temp;
            sz++; // correction due to separator ";\t"
            st += sz;
        }
    }
    std::getline(ctrlfile, line);
    c.delay = std::stof(line.substr(0), &sz);

}


void readTargetSequence(const int ntargs, std::vector<std::pair<double, double>>& targ_list, mjControl& c)
{
    for (int itarg = 0; itarg < ntargs; itarg++)
    {
        targ_list.push_back(std::make_pair(c.targ_list[itarg * 2], c.targ_list[itarg * 2 + 1]));
    }
    c.ntargs = ntargs;
}

// --- logging metadata and output ---

void makeMetaDataGeneral(std::string& filename, mjData* d, mjModel* m, logControl& L)
{
    std::ofstream metafile;
    metafile.open(filename);

    metafile << "This is the metadata file for simulation output .txt file with the corresponding name.\n";

    time_t rawtime;
    struct tm* timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    metafile << "Time of metadata generation: " << asctime(timeinfo) << "\n  \n FIELDS: columns \n+++++\n";

    int numberCols = 0;

    if (L.time)
    {
        metafile << "d->time: 1\n";
        numberCols += 1;
    }
    if (L.qpos)
    {
        metafile << "d->qpos: " << m->nq << "\n";
        numberCols += m->nq;
    }
    if (L.qvel)
    {
        metafile << "d->qvel: " << m->nv << "\n";
        numberCols += m->nv;
    }
    if (L.qacc)
    {
        metafile << "d->qacc: " << m->nv << "\n";
        numberCols += m->nv;
    }
    if (L.targ_xpos)
    {
        metafile << "targ_xpos: 2\n";
        numberCols += 2;
    }
    if (L.grip_xpos)
    {
        metafile << "grip_xpos: 2\n";
        numberCols += 2;
    }
    if (L.ctrl)
    {
        metafile << "d->ctrl: " << m->nu << "\n";
        numberCols += m->nu;
    }
    if (L.actuator_force)
    {
        metafile << "d->actuator_force: " << m->nu << "\n";
        numberCols += m->nu;
    }
    if (L.actuator_moment)
    {
        metafile << "d->actuator_moment: " << (m->nu * m->nv) << "\n";
        numberCols += (m->nu * m->nv);
    }
    if (L.actuator_length)
    {
        metafile << "d->actuator_length: " << m->nu << "\n";
        numberCols += m->nu;
    }
    if (L.actuator_velocity)
    {
        metafile << "d->actuator_velocity: " << m->nu << "\n";
        numberCols += m->nu;
    }
    if (L.gain_len)
    {
        metafile << "c.gain_len: " << m->nu << "\n";
        numberCols += m->nu;
    }
    if (L.gain_vel)
    {
        metafile << "c.gain_vel: " << m->nu << "\n";
        numberCols += m->nu;
    }
    if (L.act)
    {
        metafile << "d->act: " << m->nu << "\n";
        numberCols += m->nu;
    }

    metafile << "-----\n Total number of columns: " << numberCols << "\n";
    metafile << " Column separator: ;\\t" ;
    metafile.close();

    L.metaIsDone = 1;
}

void logGeneral(std::ofstream& outfile, mjData* d, mjModel* m, mjControl& c, logControl& L, std::string& filename)
{
    if (!L.metaIsDone)
        makeMetaDataGeneral(filename, d, m, L);

    int grpid = mj_name2id(m, mjOBJ_SITE, "grip");

    if (L.time)
        outfile << std::fixed << std::setprecision(12) << d->time;

    if (L.qpos)
    {
        for (int i = 0; i < m->nq; i++)
            outfile << std::fixed << std::setprecision(12) << L.separator << d->qpos[i];
    }

    if (L.qvel)
    {
        for (int i = 0; i < m->nv; i++)
            outfile << std::fixed << std::setprecision(12) << L.separator << d->qvel[i];
    }

    if (L.qacc)
    {
        for (int i = 0; i < m->nv; i++)
            outfile << std::fixed << std::setprecision(12) << L.separator << d->qacc[i];
    }

    if (L.targ_xpos)
    {
        outfile << std::fixed << std::setprecision(12) << ";\t" << c.targ_xpos[0] << ";\t" << c.targ_xpos[1];
    }

    if (L.grip_xpos)
    {
        outfile << std::fixed << std::setprecision(12) << ";\t" << d->site_xpos[grpid * 3] << ";\t" << d->site_xpos[grpid * 3 + 1];
    }

    if (L.ctrl)
    {
        for (int i = 0; i < m->nu; i++)
            outfile << std::fixed << std::setprecision(12) << ";\t" << d->ctrl[i];
    }

    if (L.actuator_force)
    {
        for (int i = 0; i < m->nu; i++)
            outfile << std::fixed << std::setprecision(12) << ";\t" << d->actuator_force[i];
    }

    if (L.actuator_moment)
    {
        for (int i = 0; i < (m->nu * m->nv); i++)
            outfile << std::fixed << std::setprecision(12) << ";\t" << d->actuator_moment[i];
    }

    if (L.actuator_length)
    {
        for (int i = 0; i < m->nu; i++)
            outfile << std::fixed << std::setprecision(12) << ";\t" << d->actuator_length[i];
    }

    if (L.actuator_velocity)
    {
        for (int i = 0; i < m->nu; i++)
            outfile << std::fixed << std::setprecision(12) << ";\t" << d->actuator_velocity[i];
    }

    if (L.gain_len)
    {
        for (int i = 0; i < m->nu; i++)
            outfile << std::fixed << std::setprecision(12) << ";\t" << c.gain_len[i];
    }

    if (L.gain_vel)
    {
        for (int i = 0; i < m->nu; i++)
            outfile << std::fixed << std::setprecision(12) << ";\t" << c.gain_vel[i];
    }

    if (L.act)
    {
        for (int i = 0; i < m->nu; i++)
            outfile << std::fixed << std::setprecision(12) << ";\t" << d->act[i];
    }


    outfile << "\n";
}
