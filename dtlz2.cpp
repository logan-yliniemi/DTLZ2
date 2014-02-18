
/*
 
 */

#define DO_NSGA 0
#define DO_SPEA 0

#include <cstdlib>
#include <math.h>
#include <time.h>
#include <algorithm>
#include <numeric>

#include <stdio.h>
#include <iostream>
using namespace std;

#ifndef VECTOR_INCLUDE
#define VECTOR_INCLUDE
#include <vector>
#endif

#define PI 3.1415

void report(FILE* pFILE, double value, int tabindicator);
void newline(FILE* pFILE);

/// Small Functions/Macros
#define LYRAND (double)rand()/RAND_MAX
double LYrand_norm(double a){
    
    double theta=LYRAND*2*PI;
    double rsq=-1*a*log(LYRAND);
    double x=rsq*cos(theta);   
    return(x);
}
#define SMALL 0.0001
#define HUNDI_ROUND(x) (double)floor(x*100)/100

/// Problem Domain Parameters
#define XMAX 10
#define YMAX 11
#define XSTATES 10
#define YSTATES 11
#define TOTAL_STATES (XSTATES * YSTATES)
#define VELOCITY 1
#define ACTIONS 5


/// Evoluationary Algorithm Parameters
/// <PARAM>
#define POPULATION 5
#define ELIMINATE 2
#define GENERATIONS 150000
#define STEPS 1
#define STAT_RUNS 1
#define BETA 0.5

vector< vector<double> > Anchors;

#include "Procedural_Transformation.h"
#include "Evo_Agent_DTLZ2.h"
#include "NSGAheader.h"
#include "SPEAheader.h"

bool pretty_print = true;

using namespace std;

double vector_median(vector<double> fit){
    double median;
    /// sort vector
    sort(fit.begin(),fit.end());
    /// even or odd size
    int even = (fit.size()+1)%2;
    int odd = fit.size()%2;
    if(even){ /// even case
        int mid = fit.size()/2;
        double m1 = fit.at(mid);
        double m2 = fit.at(mid+1);
        median = (m1+m2)/2;
    }
    if(odd){ /// odd case
        int mid = fit.size()/2;
        median = fit.at(mid);
    }
    return median;
}
double vector_mean(vector<double> fit){
    double sum=accumulate(fit.begin(), fit.end(), 0.0);
    double mean = sum / fit.size();
    return mean;
}

void report(FILE* pFILE, double value, int tabindicator) { /// report to text file
    fprintf(pFILE, "%.5f", value);
    if(tabindicator){
        fprintf(pFILE,"\t");
    }
}
void newline(FILE* pFILE) { /// report to text file
    fprintf(pFILE, "\n");
}

class DTLZ2class{
public:
    vector<double> function_vals;
    void eval(vector<double>,vector<double>);
    bool validity_check(vector<double>);
    double g_eval(vector<double>);
    double start();
};

bool DTLZ2class::validity_check(vector<double> soln){
    /// unused 2/18/14
    for(int i=0; i<soln.size(); i++){
        if(soln.at(i) < 0){
            return false;
        }
        if(soln.at(i) > 1){
            return false;
        }
    }
    return true;
}


void DTLZ2class::eval(vector<double> soln, vector<double> xM){
    /// soln size OBJECTIVES - 1
    /// xM size dependent on difficulty
    
    function_vals.clear();
    
    /// TODO GENERALIZE;
    double g = g_eval(xM);
    double value;
    value = 1+g;
    value*=cos(soln.at(0)*PI/2);
    value*=cos(soln.at(1)*PI/2);
    function_vals.push_back(value);
    
    value = 1+g;
    value*=cos(soln.at(0)*PI/2);
    value*=sin(soln.at(1)*PI/2);
    function_vals.push_back(value);
    
    value = 1+g;
    value*=sin(soln.at(0)*PI/2);
    function_vals.push_back(value);
    return;
}
        
double DTLZ2class::g_eval(vector<double> xM){
    /// TODO remove redundant calls.
    double g=0,summat=0;
    for(int i=0; i<xM.size(); i++){
        double xi = xM.at(i);
        double t1 = pow(xi - 0.5,2);
        double t2 = 0;
        summat += t1-t2;
    }
    g=summat;
    return g;
}

double DTLZ2class::start(){
    /// TODO if needed.
}

int main(){ /// DTLZ2
    srand(time(NULL));
    FILE* pFILE_fit;
    FILE* pFILE_f1;
    FILE* pFILE_f2;
    FILE* pFILE_f3;
    FILE* pFILE_pareto_number;
    FILE* pFILE_pareto_front;
    pFILE_fit=fopen("fitness.txt","w");
    pFILE_f1=fopen("f1.txt","w");
    pFILE_f2=fopen("f2.txt","w");
    pFILE_f3=fopen("f3.txt","w");
    pFILE_pareto_number=fopen("pareto_size.txt","w");
    
    Procedural_Transformation T;
    SPEA_2 SPEA;
    
    vector<double> one;
    vector<double> two;
    vector<double> three;
    vector<double> baddie;
    one.push_back(-1); 
    one.push_back(0);
    one.push_back(0);
    two.push_back(0);
    two.push_back(-1);
    two.push_back(0);
    three.push_back(0);
    three.push_back(0);
    three.push_back(-1);
    /// Lower bound of feasible points.
    baddie.push_back(-2);
    baddie.push_back(-2);
    baddie.push_back(-2);
    
    /// <LYLY PARAM>
    //Anchors.push_back(one);
    //Anchors.push_back(two);
    //Anchors.push_back(three);
    
    for(int stat_run=0; stat_run < STAT_RUNS; stat_run++) {
            T.Pareto_Reset();
            T.Pareto_Check(baddie); /// For first iteration, so that P^*_I is not an empty set.
            
            /// <LYLY PARAM> We assume we can find our best solution for each objective.
            //T.Pareto_Check(one);
            //T.Pareto_Check(two);
            //T.Pareto_Check(three);
            cout << "POST THREE INITIAL PARETO CHECKS: " << T.get_pareto_size() << endl;
            
            DTLZ2class environment;
            environment.start();
            DTLZ2class* pE = &environment; /// pointer to Environment

            vector<Evo_Agent_DTLZ2> Agents;
            vector<Evo_Agent_DTLZ2>* pVA = &Agents; /// pointer to Vector of Agents

            for (int i = 0; i < POPULATION; i++) {
                Evo_Agent_DTLZ2 EA;
                EA.start();
                EA.id=i;
                pVA->push_back(EA);
            }

            for (int gen = 0; gen < GENERATIONS; gen++){
                vector< vector<double> > to_pareto_check;
                if (gen % (GENERATIONS / 10) == 0) {
                    cout << endl;
                    cout << "Run No." << stat_run << " is " << (double) gen / GENERATIONS * 100 << " % Complete!" << endl;
                }
                
                /// For each population member in pA, execute 1 round of the DTLZ2 domain:
                for (int mem=0; mem<POPULATION; mem++) {
                    Evo_Agent_DTLZ2* pA = &pVA->at(mem);
                    pA->reset();
                for (int time = 0; time < STEPS; time++) {
                   
                    pE->eval(pA->get_actions(),pA->get_xM());
                    /// Agent receives objective evaluations from environment.
                    pA->set_fxn(0,-pE->function_vals.at(0));
                    pA->set_fxn(1,-pE->function_vals.at(1));
                    pA->set_fxn(2,-pE->function_vals.at(2));
               }
               }
                
                /// and now we do comparisons.
                for (int a = 0; a < pVA->size(); a++) {
                    /// <PARAM>
                    /// <DTLZ Linear Combination of Objectives>
                    // pVA->at(a).fitness = (pVA->at(a).get_fxn(0))*1/3 + pVA->at(a).get_fxn(1)*(1/3) + pVA->at(a).get_fxn(2)*1/3;
                    /// <F1 Only>
                    // pVA->at(a).fitness = pVA->at(a).get_fxn(0);
                    /// <F2 Only>
                    //pVA->at(a).fitness = pVA->at(a).get_fxn(1);
                    /// <F3 Only>
                    //pVA->at(a).fitness = pVA->at(a).get_fxn(2);
                    /// <Procedural Transformation>
                    vector<double> MO;
                    vector<double>* pMO;
                    
                    pMO = &MO;
                    MO.push_back(pVA->at(a).get_fxn(0));
                    MO.push_back(pVA->at(a).get_fxn(1));
                    MO.push_back(pVA->at(a).get_fxn(2));
                    //cout << "MO1 " << MO.at(0) << endl;
                    //cout << "MO2 " << MO.at(1) << endl;
                    
                    to_pareto_check.push_back(MO);
                    T.execute_N_transform(pMO,a); /// TODO
                    

                    double TIME_WEIGHT=BETA;
                     pVA->at(a).transformed_fitness = MO.at(0) + MO.at(1) + MO.at(2);
                     pVA->at(a).fitness = pVA->at(a).transformed_fitness;
                
                
                }
                /// Generation-based "batch" pareto checking.
                for(int i=0; i<to_pareto_check.size(); i++){
                    T.Pareto_Check(to_pareto_check.at(i));
                }
                
                vector<double> fit;
                vector<double> f1;
                vector<double> f2;
                vector<double> f3;
                
                for (int a = 0; a < pVA->size(); a++) {
                    fit.push_back(pVA->at(a).get_fitness());
                    f1.push_back(- pVA->at(a).get_fxn(0));
                    f2.push_back(- pVA->at(a).get_fxn(1));
                    f3.push_back(- pVA->at(a).get_fxn(2));
                   report(pFILE_f1,f1.back(),1); // Report every result
                   report(pFILE_f2,f2.back(),1); // Report every result
                   report(pFILE_f3,f3.back(),1); // Report every result
                   report(pFILE_pareto_number,T.get_pareto_size(),1);
                }
                
                if(DO_NSGA){ /// Overwrites fitness values before selection
                    NSGA_2 NSGA;
                    NSGA.declare_NSGA_dimension(3);
                    NSGA.NSGA_reset();
                    for (int a = 0; a < pVA->size(); a++) {
                        vector<double> afit;
                        afit.push_back(pVA->at(a).get_fxn(0));
                        afit.push_back(pVA->at(a).get_fxn(1));
                        afit.push_back(pVA->at(a).get_fxn(2));
                        NSGA.vector_input(afit,a);
                    }
                    NSGA.execute();
                    for (int a = 0; a < pVA->size(); a++) {
                        pVA->at(a).fitness=NSGA.NSGA_member_fitness(a);
                    }
                }
                
                if(DO_SPEA){ /// Performs selection mechanism natively.
                    for(int a=0; a<pVA->size(); a++){
                    vector<double> MO;
                    MO.push_back(pVA->at(a).get_fxn(0));
                    MO.push_back(pVA->at(a).get_fxn(1));
                    MO.push_back(pVA->at(a).get_fxn(2));
                    SPEA.vector_input(MO,a);
                    SPEA.take_agent(pVA->at(a),a);
                    }
                    
                vector<int> survivors;
                vector<int>* pS=&survivors;
                SPEA.execute(pS);
                
                pVA->clear();
                for(int i=0; i< pS->size(); i++){
                    int el = pS->at(i);
                    pVA->push_back(SPEA.archive.at(el).agent);
                    pVA->back().mutate();
                }
                }
                
                
                /// determine mean, median for reporting
                double generation_median = vector_median(fit);
                double generation_mean = vector_mean(fit);
                /// LY Note: [median_f1,median_f2,median_f3] does not correspond to a single point.
                double median_f1 = vector_median(f1);
                double median_f2 = vector_median(f2);
                double median_f3 = vector_median(f3);
                if(gen % 100 == 0){
                cout << "Generation\t" << gen << "\tf1:\t" << median_f1 << "\tf2:\t" << median_f2 << "\tf3:\t" << median_f3 << "\tfitness:\t" << generation_median << endl;
                }
                
                if(!DO_SPEA){ /// Because SPEA2 does the selection enclosed in its own bloc.
                /// always eliminate worst-performing solutions.
                for(int e=0; e<ELIMINATE; e++){
                    double minfit = *min_element(fit.begin(), fit.end());
                    int spot = min_element(fit.begin(), fit.end()) - fit.begin();
                    // kill this fitness;
                    fit.erase(fit.begin() + spot);
                    // kill this agent
                    pVA->erase(pVA->begin() + spot);
                }
                
                /// duplicate best-performing agents
                for(int d=0; d<ELIMINATE; d++){
                    double maxfit = *max_element(fit.begin(), fit.end());
                    int spot = max_element(fit.begin(), fit.end()) - fit.begin();
                    /// <PARAM>
                    /// to reduce the rate of convergence, we select the best one (spot) 50% of the time,
                    /// and the other 50% of the time we select a random survivor.
                    if(rand()%2){spot=rand()%pVA->size();}
              
                    /// create exact copy
                    pVA->push_back(pVA->at(spot));
                    /// mutate
                    pVA->back().mutate();
                }
                
            }
                
                if (pretty_print) {
                   report(pFILE_fit,generation_mean,1); /// report every result
                   report(pFILE_pareto_number,T.get_pareto_size(),1);
               } else {                
                   //For Coarse Results
                   if (gen % (GENERATIONS / 100) == 0) {
                       report(pFILE_fit,generation_mean,1); // Report only occasionally
                       report(pFILE_pareto_number,T.get_pareto_size(),1);
                   }
               }
            
    }
            //Start a new line in output file for next run
            fprintf(pFILE_fit,"\n");
            fprintf(pFILE_f1,"\n");
            fprintf(pFILE_f2,"\n");
            fprintf(pFILE_f3,"\n");
            fprintf(pFILE_pareto_number,"\n");
            T.print_pareto(pFILE_pareto_front);
}      
    fclose(pFILE_f1);
    fclose(pFILE_f2);
    fclose(pFILE_f3);
    fclose(pFILE_fit);
    fclose(pFILE_pareto_number);
    T.cout_pareto();
    T.exhaustive_to_file();
}
