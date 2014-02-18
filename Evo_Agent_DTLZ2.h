/* 
 * File:   Evo_Agent_DTLZ2.h
 * Author: ylinieml
 *
 * Created on December 11, 2013, 1:40 PM
 */

#ifndef EVO_AGENT_DTLZ2_H
#define	EVO_AGENT_DTLZ2_H

#ifndef VECTOR_INCLUDE
#define VECTOR_INCLUDE
#include <vector.h>
#endif

class Evo_Agent_DTLZ2 {
    
    /// ACTIONS
    void create_action_vector(int OBJS, int k);
    vector<double> actions;
    vector<double> xM;
    
    /// EVALUATIONS
    vector <double> objective_evals; /// each individual objective
    vector <double> transformed_objective_evals; /// each individual objective, post-transform

    /// VECTOR OUTPUTS
    void console_1vector(vector<double>);
    void console_2vector(vector<vector<double> >);

public:
    /// ACTIONS
    vector<double> get_xM();
    vector<double> get_actions();
    double get_action_element(int time);
    double get_xM_element(int elem);
    void console_action_vector();
    double action;
    
    /// EVALUATIONS
    double transformed_fitness; /// transformed fitness after Pareto Transformation
    double fitness; /// used for actual evolutionary method
    double get_fitness();
    void set_fitness(double fit);
    void set_fxn(int dex, double val);
    double get_fxn(int dex);
    
    /// MUTATIONS
    void mutate();
    
    /// FLOW
    void start();       //Used before first run of a statistical run / repeat
    void reset();       //Used at start of episode
    int id;
};

void Evo_Agent_DTLZ2::start() {
    /// id is set outside of start, in main();
    create_action_vector(3,10);
    fitness=-1;
}

void Evo_Agent_DTLZ2::reset() { /// Episode-based reset to clear objective vectors.
    objective_evals.clear();
    objective_evals.resize(OBJECTIVES);
    transformed_objective_evals.clear();
    transformed_objective_evals.resize(OBJECTIVES);
}

void Evo_Agent_DTLZ2::create_action_vector(int OBJS, int k){
    double r1;
    actions.clear();
    xM.clear();
    for(int i=0; i<OBJS-1; i++){
        r1=(double)rand()/RAND_MAX;
        actions.push_back(r1);
    }
    for(int i=0; i<k; i++){
        r1=(double)rand()/RAND_MAX;
        xM.push_back(r1);
    }
}

double Evo_Agent_DTLZ2::get_fitness(){
    return fitness;
}

double Evo_Agent_DTLZ2::get_action_element(int time){
    return actions.at(time);
}

double Evo_Agent_DTLZ2::get_xM_element(int elem){
    return xM.at(elem);
}

vector<double> Evo_Agent_DTLZ2::get_xM(){
    return xM;
}
vector<double> Evo_Agent_DTLZ2::get_actions(){
    return actions;
}

void Evo_Agent_DTLZ2::set_fxn(int elem, double val){
    objective_evals.at(elem)=val;
}

double Evo_Agent_DTLZ2::get_fxn(int dex){
    return objective_evals.at(dex);
}

void Evo_Agent_DTLZ2::set_fitness(double fit){
    fitness=fit;
}

void Evo_Agent_DTLZ2::console_2vector(vector< vector<double> > a) {
    /// Output 2 dimensional vector to console.
    for (int i = 0; i < a.size(); i++) {
        console_1vector(a.at(i));
    }
    cout << endl;
}


void Evo_Agent_DTLZ2::console_1vector(vector<double> a) {
    /// Output 1 dimensional vector to console.
    for (int i = 0; i < a.size(); i++) {
        cout << a.at(i);
        cout << "\t";
    }
    cout << endl;
}

void Evo_Agent_DTLZ2::console_action_vector() {
    int A=actions.size();
    for(int a=0; a<A; a++){
        cout << actions.at(a) << "\t";
    }
    cout << " || ";
    for(int a=0; a<xM.size(); a++){
        cout << xM.at(a) << "\t";
    }
    cout << endl;
}

void Evo_Agent_DTLZ2::mutate() {
    /// Mutation operator for EA/SA algorithm
    /// <PARAM>
    for(int i=0; i<actions.size(); i++){
        actions.at(i)+=LYrand_norm(0.1);
        if(actions.at(i)<0){actions.at(i) = 0;}
        if(actions.at(i)>1){actions.at(i) = 1;}
    }
    for(int j=0; j<xM.size(); j++){
        xM.at(j)+=LYrand_norm(0.1);
        if(xM.at(j)<0){xM.at(j) = 0;}
        if(xM.at(j)>1){xM.at(j) = 1;}
    }
}

#endif	/* EVO_AGENT_DTLZ2_H */

