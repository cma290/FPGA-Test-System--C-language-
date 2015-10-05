
/*=======================================================================
A simple parser for "self" format

The circuit format (called "self" format) is based on outputs of
a ISCAS 85 format translator written by Dr. Sandeep Gupta.
The format uses only integers to represent circuit information.
The format is as follows:

1        2        3        4           5           6 ...
------   -------  -------  ---------   --------    --------
0 GATE   outline  0 IPT    #_of_fout   #_of_fin    inlines
                1 BRCH
                2 XOR(currently not implemented)
                3 OR
                4 NOR
                5 NOT
                6 NAND
                7 AND

1 PI     outline  0        #_of_fout   0
2 FB     outline  1 BRCH   inline
3 PO     outline  2 - 7    0           #_of_fin    inlines




                                  Author: Chihang Chen
                                  Date: 9/16/94

=======================================================================*/

/*=======================================================================
- Write your program as a subroutine under main().
  The following is an example to add another command 'lev' under main()

enum e_com {READ, PC, HELP, QUIT, LEV};
#define NUMFUNCS 5
int cread(), pc(), quit(), lev();
struct cmdstruc command[NUMFUNCS] = {
 {"READ", cread, EXEC},
 {"PC", pc, CKTLD},
 {"HELP", help, EXEC},
 {"QUIT", quit, EXEC},
 {"LEV", lev, CKTLD},
};

lev()
{
 ...
}
=======================================================================*/

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>

#define MAXLINE 81               /* Input buffer size */
#define MAXNAME 31               /* File name size */
#define numOfNodes 200	//added for lev()
#define MAX_ATE_VEC 200	// max number of vectors in the ATE response file. Can be changed to dynamic later.
#define CONE_CLB_NUM 30	// max number of CLB in the cone intercection, which is possible faulty clb number. worst case this equals num of clb in circuit

#define Upcase(x) ((isalpha(x) && islower(x))? toupper(x) : (x))
#define Lowcase(x) ((isalpha(x) && isupper(x))? tolower(x) : (x))

enum e_com {READ, PC, HELP, QUIT, LEV, ATE};  //read ate response file
enum e_state {EXEC, CKTLD};         /* Gstate values */
enum e_ntype {GATE, PI, FB, PO};    /* column 1 of circuit format */  // node type
//enum e_gtype {IPT, BRCH, XOR, OR, NOR, NOT, NAND, AND};  /* gate types */ //CLB here

struct cmdstruc {
	char name[MAXNAME];        /* command syntax */
	int (*fptr)();             /* function pointer of the commands */
	enum e_state state;        /* execution state sequence */
};

typedef struct n_struc {
	unsigned indx;             /* node index(from 0 to NumOfLine - 1 */
	unsigned num;              /* line number(May be different from indx */
	int type;         /* gate type */  // CLB 0 - 255, PI 259,FF256
	unsigned fin;              /* number of fanins */
	unsigned fout;             /* number of fanouts */
	struct n_struc **unodes;   /* pointer to array of up nodes */
	struct n_struc **dnodes;   /* pointer to array of down nodes */

	int level;                 /* level of the gate output */
	int NumInpsReady;		//added to reduce complexity of inputLevelize()

	int clbBinary[8];       // CLB type number in binary.
	int input1[MAX_ATE_VEC],input2[MAX_ATE_VEC],input3[MAX_ATE_VEC]; //MSB input1 LSB input3.. ate input vectors
	int output[MAX_ATE_VEC]; // logic simulation results of ate vectors
	int fltyOut[CONE_CLB_NUM][MAX_ATE_VEC]; //CONE_CLB_NUM:clb num in cone intersection
	//int faultID[CONE_CLB_NUM][9];
	int ATEbits[MAX_ATE_VEC]; // for PIs and POs, test vectors and responses, for FF this is initial values
	int cone[MAX_ATE_VEC];//need bits to mark cone intersection. one elemt for each test vector
  int ft[256]; // ing distance wrt the ATE POs.
} NSTRUC;


int coneMax[MAX_ATE_VEC]; //track the max number of cone stack counter, 
int numOfVec;
int k=0;//index of ready line. also appeared as 
NSTRUC *readyline[numOfNodes]; //since it is global, can not use Nnodes.
int maxLev; //for simulation use. The for loop there needs a maxlev


/*----------------- Command definitions ----------------------------------*/
#define NUMFUNCS 6
int cread(), pc(), help(), quit(), lev(), read_ate();
struct cmdstruc command[NUMFUNCS] = {
		{"READ", cread, EXEC},
		{"PC", pc, CKTLD},
		{"HELP", help, EXEC},
		{"QUIT", quit, EXEC},
		{"LEV", lev, CKTLD},
		{"ATE",read_ate, CKTLD },  // maybe should be cktld?
};

/*------------------------------------------------------------------------*/
enum e_state Gstate = EXEC;     /* global exectution sequence */
NSTRUC *Node;                   /* dynamic array of nodes */
NSTRUC **Pinput;                /* pointer to array of primary inputs */    //need to add PPI later
NSTRUC **Poutput;               /* pointer to array of primary outputs */   //need to add PPO later
int Nnodes;                     /* number of nodes */
int Npi;                        /* number of primary inputs */
int Npo;                        /* number of primary outputs */
int Done = 0;                   /* status bit to terminate program */
int Numnodes, Numpi;
/*------------------------------------------------------------------------*/




/*-----------------------------------------------------------------------
input: nothing
output: nothing
called by: shell
description:
This is the main program of the simulator. It displays the prompt, reads
and parses the user command, and calls the corresponding routines.
Commands not reconized by the parser are passed along to the shell.
The command is executed according to some pre-determined sequence.
For example, we have to read in the circuit description file before any
action commands.  The code uses "Gstate" to check the execution
sequence.
Pointers to functions are used to make function calls which makes the
code short and clean.
-----------------------------------------------------------------------*/
main()
{
	enum e_com com;
	char cline[MAXLINE], wstr[MAXLINE], *cp; //

	while(!Done) {
		printf("\nCommand>");
		fgets(cline, MAXLINE, stdin); //from keyboard
		if(sscanf(cline, "%s", wstr) != 1) continue; // process what we got from the commandline
		cp = wstr;
		while(*cp){
			*cp= Upcase(*cp);
			cp++;
		}
		cp = cline + strlen(wstr);
		com = READ;
		while(com < NUMFUNCS && strcmp(wstr, command[com].name)) com++;
		if(com < NUMFUNCS) {
			if(command[com].state <= Gstate) (*command[com].fptr)(cp);
			else printf("Execution out of sequence!\n");
		}
		else system(cline);
	}
}

/*-----------------------------------------------------------------------
input: circuit description file name
output: nothing
called by: main
description:
This routine reads in the circuit description file and set up all the
required data structure. It first checks if the file exists, then it
sets up a mapping table, determines the number of nodes, PI's and PO's,
allocates dynamic data arrays, and fills in the structural information
of the circuit. In the ISCAS circuit description format, only upstream
nodes are specified. Downstream nodes are implied. However, to facilitate
forward implication, they are also built up in the data structure.
To have the maximal flexibility, three passes through the circuit file
are required: the first pass to determine the size of the mapping table
, the second to fill in the mapping table, and the third to actually
set up the circuit information. These procedures may be simplified in
the future.
-----------------------------------------------------------------------*/
cread(cp)
char *cp;
{
	char buf[MAXLINE];
	int ntbl, *tbl, i, j, k, nd, tp, fo, fi, ni = 0, no = 0; //node table, node
	FILE *fd;
	NSTRUC *np;

	sscanf(cp, "%s", buf);
	if((fd = fopen(buf,"r")) == NULL) {
		printf("File %s does not exist!\n", buf);
		return;
	}
	if(Gstate >= CKTLD) clear();
	Nnodes = Npi = Npo = ntbl = 0;
	while(fgets(buf, MAXLINE, fd) != NULL) {
		if(sscanf(buf,"%d %d", &tp, &nd) == 2) {
			if(ntbl < nd) ntbl = nd;
			Nnodes ++;
			if(tp == PI) Npi++;
			else if(tp == PO) Npo++;
		}
	}
	tbl = (int *) malloc(++ntbl * sizeof(int));
	Numpi=Npi;
	fseek(fd, 0L, 0);
	i = 0;
	while(fgets(buf, MAXLINE, fd) != NULL) {
		if(sscanf(buf,"%d %d", &tp, &nd) == 2) tbl[nd] = i++;
	}
	allocate();

	fseek(fd, 0L, 0);
	while(fscanf(fd, "%d %d", &tp, &nd) != EOF) {
		np = &Node[tbl[nd]];
		np->num = nd;
		if(tp == PI) Pinput[ni++] = np;
		else if(tp == PO) Poutput[no++] = np;
		switch(tp) {
		case PI:	// fin=0
		case PO:	// fin= fin of its gate
		case GATE:
			fscanf(fd, "%d %d %d", &np->type, &np->fout, &np->fin);
			break;

		case FB:
			np->fout = np->fin = 1;
			fscanf(fd, "%d", &np->type);
			break;

		default:
			printf("Unknown node type!\n");
			exit(-1);
		}
		np->unodes = (NSTRUC **) malloc(np->fin * sizeof(NSTRUC *));
		np->dnodes = (NSTRUC **) malloc(np->fout * sizeof(NSTRUC *));
		for(i = 0; i < np->fin; i++) {
			fscanf(fd, "%d", &nd);
			np->unodes[i] = &Node[tbl[nd]];
		}
		for(i = 0; i < np->fout; np->dnodes[i++] = NULL);
	}
	Numnodes = Nnodes;
	for(i = 0; i < Nnodes; i++) {
		for(j = 0; j < Node[i].fin; j++) {
			np = Node[i].unodes[j];
			k = 0;
			while(np->dnodes[k] != NULL) k++;
			np->dnodes[k] = &Node[i];
		}
	}
	fclose(fd);
	Gstate = CKTLD;

	//add a initialization which fill the clbBinary[], and ft array 
	initial();
	printf("==> read circuit OK\n");
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//HW 3 TASK:

//levelization
lev()
{	
	int maxLev = 0;
	//Initialize NumInpsReady to zero
	int i,j;
	for (i = 0; i < Nnodes; i++){
		Node[i].NumInpsReady = 0;
	}
	//Then first set PI = 0 lev; and at the same time
	//increase numInpsReady of corresponding dnodes
	for (i = 0; i < Npi; i++){
		Pinput[i]->level = 0;
		if(Pinput[i]->fout > 0){ // this is always true?
			Pinput[i]->dnodes[0]-> NumInpsReady++;//picked [0] as the mark, will these downnodes have the same lev?
			if(Pinput[i]->dnodes[0]-> NumInpsReady == Pinput[i]->dnodes[0]->fin){
				for(j = 0; j < Pinput[i]->fout; j++){
					readyline[k]= Pinput[i]->dnodes[j];
					k=k++;
				}
			}
		}
	}
	if (k>0){    //call the recursive helper function
		level( readyline[k-1]);
	}
	printf("==> lev finished\n");
}

//recursive part of levelization
level( NSTRUC *ready){
	int j,i,max;
	if(ready->type == 1){ 	//branches
		ready->level = ready->unodes[0]->level +1;
		k--;
	}
	else{	//gates. PI will not be passed here. so didnt wtire (type !=0).
		max = ready->unodes[0]->level;
		for(i=0; i < ready->fin; i++){
			if(ready->unodes[i]->level > max){
				max = ready->unodes[i]->level;}
		}
		ready->level= max+1;
		k--;
		if (max + 1 > maxLev) 
		{maxLev = max + 1;}	
		printf("maxLev = %d ",maxLev);//debug use		
	}
	if(ready->fout > 0){
		ready->dnodes[0]-> NumInpsReady++;
		if(ready->dnodes[0]-> NumInpsReady == ready->dnodes[0]->fin){
			for(j = 0; j < ready->fout; j++){
				readyline[k]= ready->dnodes[j];
				k=k++;
			}
		}
	}
	if (k>0){
		level( readyline[k-1]);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/*-----------------------------------------------------------------------
input: nothing
output: nothing
called by: main
description:
The routine prints out the circuit description from previous READ command.
-----------------------------------------------------------------------*/
pc(cp)
char *cp;
{
	int i, j;
	NSTRUC *np;
	char *gname();

	printf(" Node   Type \tIn     \t\t\tOut       level\n");
	printf("------ ------\t-------\t\t\t-------   -----\n");
	for(i = 0; i<Nnodes; i++) {
		np = &Node[i];
		printf("\t\t\t\t\t");
		for(j = 0; j<np->fout; j++) printf("%d ",np->dnodes[j]->num);
		printf("\r%5d  %3d\t", np->num, (np->type));   // in this project, just CLB type number,
		for(j = 0; j<np->fin; j++) printf("%d ",np->unodes[j]->num);
		printf("\t\t\t\t");
		printf("%d", np->level);
		printf("\n");
	}
	printf("Primary inputs:  ");
	for(i = 0; i<Npi; i++) printf("%d ",Pinput[i]->num);
	printf("\n");
	printf("Primary outputs: ");
	for(i = 0; i<Npo; i++) printf("%d ",Poutput[i]->num);
	printf("\n\n");
	printf("Number of nodes = %d\n", Nnodes);
	printf("Number of primary inputs = %d\n", Npi);
	printf("Number of primary outputs = %d\n", Npo);
	//debug use
	//printf("PI level %d %d", Pinput[0]->level,Pinput[1]->level);
	//printf("numinpsready bit %d %d", Node[0].NumInpsReady,Pinput[1]->NumInpsReady);
	//printf("dnodes %d %d", Pinput[0]->dnodes[0]->num,Pinput[1]->dnodes[0]->num);

}

/*-----------------------------------------------------------------------
input: nothing
output: nothing
called by: main
description:
The routine prints out help information for each command.
-----------------------------------------------------------------------*/
help()
{
	printf("READ filename - ");
	printf("read in circuit file and creat all data structures\n");
	printf("PC - ");
	printf("print circuit information\n");
	printf("HELP - ");
	printf("print this help information\n");
	printf("QUIT - ");
	printf("stop and exit\n");
}

/*-----------------------------------------------------------------------
input: nothing
output: nothing
called by: main
description:
Set Done to 1 which will terminates the program.
-----------------------------------------------------------------------*/
quit()
{
	Done = 1;
}

/*======================================================================*/

/*-----------------------------------------------------------------------
input: nothing
output: nothing
called by: cread
description:
This routine clears the memory space occupied by the previous circuit
before reading in new one. It frees up the dynamic arrays Node.unodes,
Node.dnodes, Node.flist, Node, Pinput, Poutput, and Tap.
-----------------------------------------------------------------------*/
clear()
{
	int i;

	for(i = 0; i<Nnodes; i++) {
		free(Node[i].unodes);
		free(Node[i].dnodes);
	}
	free(Node);
	free(Pinput);
	free(Poutput);
	Gstate = EXEC;
}

/*-----------------------------------------------------------------------
input: nothing
output: nothing
called by: cread
description:
This routine allocatess the memory space required by the circuit
description data structure. It allocates the dynamic arrays Node,
Node.flist, Node, Pinput, Poutput, and Tap. It also set the default
tap selection and the fanin and fanout to 0.
-----------------------------------------------------------------------*/
allocate()
{
	int i;

	Node = (NSTRUC *) malloc(Nnodes * sizeof(NSTRUC));
	Pinput = (NSTRUC **) malloc(Npi * sizeof(NSTRUC *));
	Poutput = (NSTRUC **) malloc(Npo * sizeof(NSTRUC *));
	for(i = 0; i<Nnodes; i++) {
		Node[i].indx = i;
		Node[i].fin = Node[i].fout = 0;
	}
}



//====================================== Diagnosis =========================================
//====================================== Diagnosis =========================================
//====================================== Diagnosis =========================================
//====================================== Diagnosis =========================================
//====================================== Diagnosis =========================================


// 1. Read in ATE file
//ATE response format
//	110000000---0
//	***
//	000101010---1

read_ate(cp)
char *cp;
{
	char buf[MAXLINE];
	int i,vecNum; // 
	FILE *fp;
	char c;
	
	//some initialization needed. so can reread on one run
	sscanf(cp, "%s", buf); // buf: file name
	if((fp = fopen(buf,"r")) == NULL) {
		printf("File %s does not exist!\n", buf);
		return;
	}

	vecNum = 0;
	c = fgetc(fp);
	while(c != EOF ) {
		for ( i = 0 ; i < Npi; i++){
			sscanf(&c, "%d", &Pinput[i]-> ATEbits[vecNum]);
			//printf("vecter%d PI%d : %d \n",vecNum, i, Pinput[i]-> ATEbits[vecNum]);//debug use
			c = fgetc(fp);			
		}

		while(c == '-'){
			//printf("Skiped : %c \n", c);//debug use
			c = fgetc(fp);			
		} 
		//printf("\n");
		
		for ( i = 0 ; i < Npo; i++){
			sscanf(&c, "%d", &Poutput[i]-> ATEbits[vecNum]);
			//printf("vecter%d PO%d : %d \n",vecNum, i, Poutput[i]-> ATEbits[vecNum]);
			c = fgetc(fp);
		}
		//printf("======================\n");

		if (c!=EOF){
			while(c!='0' && c!='1'){
				//printf("Skiped : %c \n", c);//debug use
				c = fgetc(fp);			
			} 
		}		
		numOfVec = vecNum+1;
		vecNum++; // do while for the next test vector
	}	
	fclose(fp);
	printf("==> read_ate - done\n\n");
	
	//function tests	
	logic_simu();
	printf("Logic_simulation finished\n\n");//debug use
	
	FILE *fp1;
	fp1 = fopen("error_matrix.txt", "w+");
	
	for(i=0; i < numOfVec; i++){
		cone_insert(i, fp1);
		//printf("\n");
		fprintf( fp1,"\n");
	}
	printf("error_matrix.txt produced\n");
	fclose(fp1);
	
	// now, ft array has the final fault "frequency", pick the top five
	int j,k,topfive;
	topfive= 0;
	FILE *fp2;
	fp2 = fopen("diagnosis_result.txt", "w+");
	
	//printf("Topfive possible faults:\n");
	fprintf( fp2,"Topfive possible faults:\n");
	for(k=numOfVec;k>0; k--){
		if(topfive < 5){
			for(i = 0;i<Nnodes;i++){
				if(topfive < 5 && Node[i].type >= 0 && Node[i].type <= 255){
					for(j = 0;j<256;j++){
						if(topfive < 5 && Node[i].ft[j] == k){
						topfive++;
						//printf("CLB %d %d freq:%d\n", Node[i].num, j,k);
						fprintf( fp2,"CLB %d %d freq:%d\n", Node[i].num, j,k);
						}
					}				
				}
			}
			
		}	
	}
	//printf("\n");
	fprintf( fp2,"\n");
	printf("diagnosis_result.txt produced\n");
	fclose(fp2);
//for(k=0;k<256;k++){printf("%d",Node[9].ft[k]);}//debug
}


	
// 2. Logic simulation	
/*simulate level by level  //easy but not very efficient, can use faster algorithm
  simu all k vectors in ate file
  output: nothing. simulated PO values are stored in array during the process
 */
logic_simu() // has eval() and applyVec() in it
{	
	int i,j,k; 
	for(k = 0; k < numOfVec; k++){
		applyVec(k);	//assign the kth input vector to PIs
		for(i = 1; i <= maxLev; i++)//simu from level 1. PI has level 0
		{
			for ( j = 0; j < Nnodes; j++)
			{
				if (Node[j].level == i)
				{
					Node[j].input1[k]= Node[j].unodes[0]->output[k];
					Node[j].input2[k]= Node[j].unodes[1]->output[k];
					Node[j].input3[k]= Node[j].unodes[2]->output[k];  //assume all three inputs are connected to upnodes		
					eval(j,k);
					//printf("logic_simu(vec%d), node%d input123: %d%d%d\n",k, Node[j].num, Node[j].input1[k],Node[j].input2[k],Node[j].input3[k]);//debug use
				}
			}
		}	
		//printf("\n");
	}	
}


//3. Cone insertion, fault insertion and fault simulation
//compare logic simu results with ate response and tell which POs are incorrect(it's possible they are all correct)
//call cone() and produce cone information
//find the intersection of cones and put clb in the section into a pointer arrary.
// for the kth vector

cone_insert(int k, FILE *fp1) 
{	
	int i;
    coneMax[k] = 0;     // if mismatch on PO, cone this PO.
	for ( i = 0 ; i < Npo; i++){
		//printf("vector%d output = %d ATEbits[k]=%d \n",k,Poutput[i]-> output[k],Poutput[i]-> ATEbits[k]);//debug use
		if (Poutput[i]-> output[k] != Poutput[i]-> ATEbits[k]){
		//printf("vecter%d PO%d(line%d), ATE differs from logic simu\n",k,i,Poutput[i]->num);
		cone(Poutput[i], k); 
		}
		else{
		//printf("vecter%d PO%d(line%d), ATE matches logic simu\n",k,i,Poutput[i]->num);
		}
	}
	
	int j;
	//coneMax[k] = 0;   // find coneMax for vec k
	//for ( j = 0; j <Nnodes; j++){
	//	if (Node[j].cone[k] > coneMax[k]){
	//		coneMax[k] = Node[j].cone[k];
	//	}
	//} //already taken care when cone
	
	//case 1, all POs are right.	
	if(coneMax[k] == 0){//search all clbs, flip and compare
	  
	  int i;
		fprintf( fp1,"Test sequence: ");
		for (i = 0; i < Npi; i++){ fprintf( fp1,"%d",Pinput[i]->ATEbits[k]);}
		fprintf( fp1,"\n");	  
	  
	  for (i = 0; i < Nnodes; i++){
				if(Node[i].type >= 0 && Node[i].type <= 255) {
					Node[i].output[k] =  (Node[i].output[k] == 1) ? 0:1; //flip the output of the chosen faulty clb
					int line, fault, in, hamming;
					line = Node[i].num;
					in = 4*Node[i].input1[k] + 2*Node[i].input2[k] + 1*Node[i].input3[k]; // input decimal, 0 to 7
					fault = (Node[i].output[k] == 1) ?(Node[i].type + ( 1<< in )):( Node[i].type - ( 1<< in ) ); //if is 1 now, means before is 0, so CLB type plus the corresponding value
					
					simuDownCone(&Node[i],k);// now output is fault simulation result
					
					hamming = 0;
					for(j=0; j < Npo; j++){
						if (Poutput[j]->output[k] != Poutput[j]->ATEbits[k]){ 
							hamming++;
						} //output now contains fault simu result.
					}
					
					if (hamming == 0){ //means match
						Node[i].ft[fault]++; //= Node[i].ft[fault]+1;
						fprintf( fp1,"Possible faults: CLB %d %d POs:",line,fault);
						//printf("Possible faults: CLB %d %d bit%d POs:",line,fault, in);
						for(j=0; j < Npo; j++){
							fprintf( fp1,"%d",Poutput[j]->output[k]); //print POs
						}
						fprintf( fp1,"\n");
					}

					//logic_simu();//logic simulation to reset all node output bits to fault free value, since my fault simu is base on logic simu
					// this simu could be replaced by?
					Node[i].output[k] =  (Node[i].output[k] == 1) ? 0:1;
					simuDownCone(&Node[i],k); // reset to logic simu results
				}
		}
	}

	
	else{
		//case 2, not all POs are right
		int clbNum = 0;  // find how many clb are there in the intersection of cones, for mallocate.
		int i,j;
		for ( j = 0; j < Nnodes; j++){
			if (coneMax[k] > 0 && Node[j].cone[k] == coneMax[k] ){   // if all outputs match, all cone=coneMax=0, in this case no intersection
				clbNum++;
			}
		}
		
		NSTRUC **intersec;
		if(clbNum > 0) {
			intersec = (NSTRUC **) malloc(clbNum * sizeof(NSTRUC *));
			i = 0; //put nodes into pointer array
			for ( j = 0; j < Nnodes; j++){
				if (Node[j].cone[k] == coneMax[k]){
					intersec[i] = &Node[j];
					i++;
					//printf("node%d cblNum%d\n",Node[j].num,clbNum);
				}
			}
		}	
		
		fprintf( fp1,"Test sequence: ");
		for (i = 0; i < Npi; i++){ fprintf( fp1,"%d",Pinput[i]->ATEbits[k]);}
		fprintf( fp1,"\n");
		
		//insert fault and fault simu
		if (clbNum>0){
			int i;
			for (i = 0; i < clbNum; i++){
				intersec[i]-> output[k] =  (intersec[i]-> output[k] == 1) ? 0:1; //flip the output of the assumed faulty clb
				
				int line, fault, in, hamming;
				line = intersec[i]-> num;
				in = 4*intersec[i]->input1[k] + 2*intersec[i]->input2[k] + 1*intersec[i]->input3[k]; 
				fault = (intersec[i]-> output[k] == 1) ?(intersec[i]->type + ( 1<< in )):( intersec[i]->type - ( 1<< in ) ); //if is 1 now, means before is 0, so CLB type plus the corresponding value
				
				simuDownCone(intersec[i],k);
				//free (intersec);	
				//printf("fault simulation(vec%d), node%d input123: %d%d%d\n",k, Poutput[0]->num, Poutput[0]->input1[k],Poutput[0]->input2[k],Poutput[0]->input3[k]);//debug use
				hamming = 0;
				int j;
				for(j=0; j < Npo; j++){
					
					if (Poutput[j]->output[k] != Poutput[j]->ATEbits[k]){ 
						hamming++;
					} //output now contains fault simu result.
					//printf("vector%d: CLB line%d type%d  PO%d=%d\n",k,line,fault, j, Poutput[j]->output[k]);//debug use		
				}
				
				if (hamming == 0){
					intersec[i]->ft[fault]++;
					fprintf( fp1,"Possible faults: CLB %d %d POs:",line,fault); // CLB line num, faulty type
					//printf("Possible faults: CLB %d %d bit%d POs:",line,fault, in); // CLB line num, faulty type
					int j;
					for(j=0; j < Npo; j++){
						fprintf( fp1,"%d",Poutput[j]->output[k]); //print POs
					}
					fprintf( fp1,"\n");
				}
				//else{ // does not exactly match with ATE POs. So these bits are impossible fault for this CUT.
				//	printf("   correct bit: CLB line%d type%d bit%d POs:",line,fault, in);//impossible fault from this ATE. printed POs are results when correct bit is fliped
				//	int j;
				//	for(j=0; j < Npo; j++){
				//		printf("%d",Poutput[j]->output[k]); //print POs
				//	}
				//	printf("\n");
				//}
	
				//logic_simu();//logic simulation to reset all node output bits to fault free value, since my fault simu is base on logic simu
				// this simu could be replaced by:
				intersec[i]-> output[k] =  (intersec[i]-> output[k] == 1) ? 0:1;
				simuDownCone(intersec[i],k);
			
			}
		}
	}
} 








//small helper functions------------------------------------------------------------------------

// 1.applyVec
//apply ate test vector number k from the previous constructed ATEbits[k] to output[k] of PI node
applyVec(int k){
int i;
	for ( i = 0 ; i < Npi; i++){
		Pinput[i]-> output[k] = Pinput[i]-> ATEbits[k];
		//printf("vec%d PI%d output=%d ATEbits=%d\n",k, i, Pinput[i]-> output[k], Pinput[i]-> ATEbits[k]);
	}		
}


// 2.cone
//find variable "cone" of up cone of a given node and increment, cone++.
// cone (Poutput[0])
cone(NSTRUC *np, int k){
int i = 0;
	if(np->type >= 0 && np->type <= 255) np->cone[k]++; // only count CLB, since only has CLB fault
	if(np->cone[k] > coneMax[k]) coneMax[k] = np->cone[k];
	if(np->fin > 0){ // has up nodes
		for( i = 0; i < np->fin; i++){
			cone(np->unodes[i],k);	//recursive: go do cone on this up nodes
		}
	}
}


// 3.simuDownCone
// simulate the down cone from a given node
simuDownCone( NSTRUC *np, int k){
	if ( np-> fout > 0 ){ 	// if no fanout, means this is alreay PO, we have fliped the output of this node
	int i;
		for (i=0; i < np->fout; i++){
			np->dnodes[i]->input1[k] = np->dnodes[i]->unodes[0]->output[k];
			np->dnodes[i]->input2[k] = np->dnodes[i]->unodes[1]->output[k];
			np->dnodes[i]->input3[k] = np->dnodes[i]->unodes[2]->output[k];
			int index = np->dnodes[i]-> indx;
			eval(index,k);  //logic simulation results no needed anymore, so just put into output. even if we need them we can just redu logic_simu()
			simuDownCone(np->dnodes[i],k);		//but now the output array size should be at least clbNum in intersection
		}   
	}       
}


// 4.eval
//evaluate a node. function input is the node index, and the vector number k.
//CLB, FB 260, PI 259, PO, CLB(FF) 256
eval(int index, int k)  //
{
	if (Node[index].type >= 0 && Node[index].type <= 255 ){
		int input = 4*Node[index].input1[k] + 2*Node[index].input2[k] + 1*Node[index].input3[k]; //decimal value of 3-bit input
		Node[index].output[k] = Node[index].clbBinary[input];  //decimal value of input as index
	}
	//else if (Node[index].type = 256 ) //FF. consider it later on
	else if (Node[index].type = 260){  // output = input, FB pass the value, without any faults
		Node[index].output[k] = Node[index].input1[k];// suppose for clb which has one input, it is "at least" connected to terminal input1.
	}
}


// 5.initial
// convert every CLB type into binary and put into clbBinary array
// reset cone = 0;
initial()  
{
	int i,j;
	int type;
	for ( i = 0; i <Nnodes; i++)
	{
		type = Node[i].type;
		if(type < 256)
		{
			int quotient = type;// decimal to binary
			int j=0;
			while (quotient!=0)
			{
				Node[i].clbBinary[j]= quotient % 2; //elemt[7] stores MSB bit7
				//printf("%d", Node[i].clbBinary[j]);//debug use
				quotient = quotient / 2;
				j = j+1;
			}
			//printf("\n");//debug use
		}
		//ini ft array
		//for ( j=0; j<256;j++){
		//	Node[i].ft[j]= 0;
		//}
		//ini cone
		for ( j=0; j<MAX_ATE_VEC;j++){
			Node[i].cone[j]= 0;
		}
	}
}



































/*========================= End of program ============================*/

