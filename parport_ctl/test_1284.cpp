// N.B! 
// http://forums.gentoo.org/viewtopic-t-619733-view-next.html?sid=923d2e9be5d55e4349eacc642a43a75f
// you'll need to rmmod lp

 
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <math.h>
#include <ieee1284.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/types.h>
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <errno.h>
#include <fcntl.h> 
// #include <time.h>
#include <map>
#include <boost/multi_array.hpp>
#include "matStor.h"
#include "sock.h"
 
#define DELAY 1000
#define NAX 9


using namespace std ;

int		g_die = false; 
//goal: allow blended movements on each axis. 
//how: keep track of  target times, positions, etc. 
int g_cnt[NAX]; //the position, in steps, along each axis. 
// this is subtracted from the reported position, and added to any move.  (e.g. communications through socket). 
// it must be initialized when the robot is turned on. 
int g_target[NAX]; //the target of the move.
double g_a[NAX]; //the accelerations on each axis. 
double g_v[NAX]; //likewise, velocity. 
double g_x[NAX]; //likewise, distance to be moved - *not* the position of the axis.
double g_at[NAX]; //acceleration time. 
double g_cvt[NAX]; //constant velocity time. 
double g_st[NAX]; //start time for a given movement. 
double g_axscl[] = {10.0, 0.2, 0.2, 0.05, 0.2, 0.2, 0.5, 0.5, 0.5}; 
int		g_limit[NAX][2]; 
unsigned int	g_data = 0; 
bool		g_reenter = false; 
int		g_controlAxis = 0; 
int		g_controlStep[] = {16,16,16,16,16,16,16,16,16}; 
bool		g_connected = false; 

int g_controlSock; //RX from controller.ml
int g_statusSock; //TX to controller
struct sockaddr_in g_statusAddr;
int g_positionSock; //TX to controller
struct sockaddr_in g_positionAddr;

char* g_axesLabels[] = {"Needle Rot", // 0
			"Cartridge Y", 	// 1
			"Cartridge X", 	// 2
			"Needle Z", 		// 3
			"Brake Rot", 		// 4
			"Brake Z", 			// 5
			"Gantry X", 		// 6
			"Gantry Y", 		// 7
			"Gantry Z"}; 		// 8

#define MAXV 25000.0  //units: counts / sec. 
#define MAXA 100000.0  // units: counts / sec ^ 2.  
#define clamp(min,x,max) ( x > min ? (x < max ? x : max) : min )
 
GtkAdjustment* g_positionSpin[NAX];
GtkWidget* g_limitEntry[NAX][2];
GtkWidget* g_da;
GtkWidget* g_window; 

void list_capabilities(int cap);

double gettime(){
	timespec pt ; 
	clock_gettime(CLOCK_MONOTONIC, &pt); 
	double ret = (double)(pt.tv_sec) ; 
	ret += (double)(pt.tv_nsec) / 1e9 ; 
	return ret ; 
	//printf( "present time: %d s %d ns \n", pt.tv_sec, pt.tv_nsec ) ; 
}

void parClose(int ){
	g_die = true; 
	sleep(1); 
}

void init_globals(){
	g_die = false; 
	MatStor ms("prefs.mat"); 
	for(int i=0; i<NAX; i++){
		g_cnt[i] = 0; 
		g_target[i] = 0; 
		g_a[i] = 0.0; 
		g_v[i] = 0.0; 
		g_at[i] = 0.0; 
		g_cvt[i] = 0.0; 
		g_st[i] = 0.0; 
		g_positionSpin[i] = 0; 
		for(int j=0; j<2; j++){
			g_limitEntry[i][j] = 0; 
			g_limit[i][j] = (int)ms.getValue2(i, j, "limit", 0.f); 
		}
	}
}
struct parport_list pl;
struct parport *port[2];

void parport_init (){
	int cap;
	int errcode = 0;
 
	printf("Finding the port\n");
	errcode = ieee1284_find_ports(&pl, 0); 
	if(errcode != E1284_OK){
		printf("Failed to find any ports.\n");
		switch(errcode){
			case E1284_NOMEM        : printf("There is not enough memory.\n");  break;
			case E1284_NOTIMPL      : printf("One or more of the supplied flags is not supported in this implementation.\n");  break;
		}
		port[0] = 0; 
		port[1] = 0; 
	}
	else{
		for(int pp = 0; pp<2 && pp < pl.portc; pp++){
			port[pp] = pl.portv[pp];
			/* open the port */
			printf("Opening the port\n");
			errcode = ieee1284_open(port[pp], F1284_EXCL, &cap);
			if(errcode != E1284_OK){
				printf ("Failed to open port: %s\n", port[pp]->name);
				switch(errcode){
					case E1284_INIT         : printf("There was a problem during port initialization.  This could be because another driver has opened the port exclusively, or some other reason.\n"); break;
					case E1284_NOMEM        : printf("There is not enough memory.\n");  break;
					case E1284_NOTAVAIL     : printf("One or more of the supplied flags is not supported by this type of port.\n");  break;
					case E1284_INVALIDPORT  : printf("The port parameter is invalid (for instance, the port may already be open).\n"); break;
					case E1284_SYS          : perror("There  was a problem at the operating system level\n"); errcode = errno;  break;
				}
				ieee1284_close(port[pp]);
				port[pp] = 0; 
			}else{
				list_capabilities(cap);
				/* Check port capacities */
				errcode = 0;
				//if((cap&CAP1284_RAW)  == 0) { printf("The port does not support raw mode: %d\n", cap);  errcode = CAP1284_RAW;  }
				if((cap&CAP1284_BYTE) == 0) { 
					printf("The port does not support byte mode: %d\n", cap); errcode = CAP1284_BYTE; }
				if(errcode == 0){
					printf ("Port info: %s %#lx %d\n", port[pp]->name, port[pp]->base_addr, cap);
					/* claim access to the ports */
					errcode = ieee1284_claim(port[pp]);
					if(errcode != E1284_OK){
						printf ("Failed to claim the port: %s\n", port[pp]->name);
						switch(errcode){
							case E1284_NOMEM        : printf("There is not enough memory.\n"); break;
							case E1284_INVALIDPORT  : printf("The port parameter is invalid (for instance, it might not have been opened yet).\n");  break;
							case E1284_SYS          : perror("There  was a problem at the operating system level\n"); errcode = errno;  break;
						}
					}else{
						printf("Captured parallel ports.\n");
					}
				}
				ieee1284_data_dir(port[pp], 0); //all output.
			}
		}
	}
	/* safely deallocate a port list */
	ieee1284_free_ports(&pl);
}
void parport_release(){
	for(int pp=0; pp<2; pp++){
		if(port[pp]){
			ieee1284_release(port[pp]);
			ieee1284_close(port[pp]);
		}
	}
	ieee1284_free_ports(&pl);
}
void parport_out(){
	if(port[0] && port[1]){
	ieee1284_write_data(port[0], (unsigned char)(g_data & 0xff)); //8 data bits
	ieee1284_write_control(port[0], (unsigned char)((g_data>>8) & 0x0f)); //4 control bits
	ieee1284_write_data(port[1], (unsigned char)((g_data>>12) & 0xff)); //8 more data bits
	}
}

bool limithit(int ax, int cnt){
	if(ax > 0 && ax < NAX){
		if(cnt < g_limit[ax][0] && cnt > g_limit[ax][1])
			return true; 
	}
	if( ax == 0 ) return false; // no limits on needle rotation. 
	// check the y / z keepout.
	int x = g_target[6]; 
	int y = g_target[7]; 
	int z = g_target[8]; 
	int keepout = 0; 
	if(x > 13000 && x < 35000){
		if(y < 1200) keepout = 1; //don't let the needle/brake run into the cartridge edge
		if(z < -1500 && y < 1400) keepout = 2; //larger margin down lower
	}
	if(x > 9500 && x < 40000){
		if(z < -18000 && y < 4750 && z > -28000) keepout = 3; 
	}
	if(z < -2500 && y < 2800) keepout = 4; 
	if(keepout){
		printf("Error: keepout rule %d violated!\n", keepout); 
		return true; 
	}
	return false;
}

void setup_move_out(int ax, 
				double a, double v, double x, 
				double at, double cvt, int target){
	//switch the direction signals. 
	//this is why it's important that the axis was not moving before. 
	if(x > 0){
		g_data |= 0x1 <<(ax*2); 
		parport_out(); 
	} else {
		g_data &= ~(0x1 <<(ax*2)); 
		parport_out();
	}
	printf("setup_move_out: ax:%d a:%f v:%f x:%f\n", ax,a,v,x); 
	//copy.
	g_a[ax] = a; 
	g_v[ax] = v; 
	g_x[ax] = x; 
	g_at[ax] = at; 
	g_cvt[ax] = cvt; 
	g_target[ax] = target ; 
}

bool setup_move(int ax, int target, int cv){
	//first check to see if this axis is moving - 
	// if it is, then do nothing.  
	if(g_cnt[ax] != g_target[ax]){
		printf("movement commanded on axis %d while axis still moving\n", ax); 
		g_target[ax] = g_cnt[ax]; //reset, since we are stopped, to permit a new target.
		return false;
	}
	if(limithit(ax, target)) {
		printf("movement commanded axis %d to %d - beyond limits of robot.\n", ax, target); 
		g_target[ax] = g_cnt[ax]; //reset, since we are stopped, to permit a new target.
	} 
	double a,v,x,cvt,at; 
	if(cv){
		a = 0.0; //this allows us to use the normal move routine. 
		v = MAXV * g_axscl[ax] * 0.05; 
		x = (double)(target - g_cnt[ax]); 
		if(x < 0.0) v *= -1.0; 
		cvt = x / v; 
		at = 0.0; 
	}else{
		a = MAXA * g_axscl[ax]; 
		v = MAXV * g_axscl[ax];
		x = (double)(target - g_cnt[ax]); 
		if(a == 0) a *= 8; 
		if(ax == 3){ //fast retraction of needle axis! 
			if(x < 0){
				a *= 32; 
				v *= 8; 
			}else{ 
				v *= 0.6; 
			}
		}
		if(x < 0.0){
			a *= -1.0 ; 
			v *= -1.0 ; 
		}
		at = v / a ; // how long to get to max velocity. 
		cvt = ( x - at * v) / v ; //how long to stay at max velocity
		if(cvt < 0.0){
			at = sqrt( x / a ) ; //if we are accelerating all the time. 
		}
	}
	setup_move_out(ax, a, v, x, at, cvt, target); 
	return true; 
}

bool setup_move2(int ax, int target, int v){
	if(g_cnt[ax] != g_target[ax]){
		printf("movement commanded on axis %d while axis still moving\n", ax); 
		g_target[ax] = g_cnt[ax]; //reset, since we are stopped, to permit a new target.
		return false;
	}
	if(limithit(ax, target)) {
		printf("movement commanded axis %d to %d - beyond limits of robot.\n", ax, target); 
		g_target[ax] = g_cnt[ax]; //reset, since we are stopped, to permit a new target.
	} 
	double a,x,cvt,at; 
	a = MAXA * g_axscl[ax]; 
	x = (double)(target - g_cnt[ax]); 
	if(x < 0.0){
		a *= -1.0 ; 
		v *= -1.0 ; 
	}
	at = v / a ; // how long to get to max velocity. 
	cvt = ( x - at * v) / v ; //how long to stay at max velocity
	if(cvt < 0.0){
		at = sqrt( x / a ) ; //if we are accelerating all the time. 
	}
	setup_move_out(ax, a, v, x, at, cvt, target); 
	return true; 
}

void move_outp(int ax, double d){
	double cnt = (double)(g_cnt[ax]); 
	double x = g_x[ax]; 
	if( d > cnt && x > 0 && d - cnt >= 1.0 && !limithit(ax,g_cnt[ax])){
		g_data |= 0x2 << (ax * 2); //set the step pin. 
		g_cnt[ax] ++ ; 
	}
	if( d < cnt && x < 0 && cnt - d >= 1.0 && !limithit(ax,g_cnt[ax])){
		g_data |= 0x2 << (ax * 2); //set the step pin. 
		g_cnt[ax] -- ; 
	}
}
void move(int ax, double prestime){
	double d = 0.0; 
	double t = prestime - g_st[ax]; 
	double a = g_a[ax]; 
	double v = g_v[ax]; 
	double x = g_x[ax]; //distance to move. 
	double at = g_at[ax]; 
	double cvt = g_cvt[ax]; 
	double s = (double)g_target[ax] - x; //start position.
	if(cvt <= 0.0){
		if( t < at ){ //first half of acceleration
			d = a * t * t / 2.0 ; 
		} else if( t < at * 2 ){
			double t2 = t - at ; //peak velocity is a * at
			d = x / 2.0 + a * at * t2 - a * t2 * t2 / 2.0 ; 
		} else {
			d = x ; 
		}
	} else {
		if( t < at){
			d = a * t * t / 2.0 ; 
		}else if( t < at + cvt ){
			d = a * at * at / 2.0 + v * (t - at); 
		}else if( t < at * 2 + cvt ){
			double t2 = t - (at + cvt); 
			d = a * at * at / 2.0 + v * cvt + (v * t2 - a * t2 * t2 / 2.0) ; 
		} else {
			d = x; 
		}
	}
	d += s ; //add in start position.
	move_outp(ax, d); 
}

bool move_not_done(){
	bool ret = true ; 
	for(int i=0; i<NAX; i++){
		ret = ret & (g_cnt[i] == g_target[i] || limithit(i,g_cnt[i]) ) ;
	}
	return ret == false ; 
}
void update_gui(){
	// need to update the GUI. 
	g_reenter = true; 
	for(int i=0; i<NAX; i++){
		gtk_adjustment_set_value(g_positionSpin[i], (double)(g_cnt[i]));
	}
	g_reenter = false; 	
}
void move_automatic(){
	int tic = 0; 
	/* note: FIFO scheduling does NOT work better than normal scheduling!! */
	double prestime = gettime() ; 
	// start the clock.
	for(int i=0; i<NAX; i++)
		g_st[i] = prestime; 
	while(move_not_done()){
		prestime = gettime() ; 
		for(int i=0; i<NAX; i++){
			move(i, prestime); 
		}
		double pulse = gettime() ; 
		parport_out(); 
		// minimum clock pulse width for TB6560AHQ is 30us; 
		// however 10us seems fine, too. 5us does not work. 
		int n = 0; 
		while(gettime() - pulse < 10e-6){
			n++; 
		}
		g_data &= 0x55555555; // clear all step pins. 
		parport_out(); 
		tic++; 
	}
}

void move_origin(){
	auto mov = [] (int ax){
		setup_move(ax, 0, 0); 
		gtk_adjustment_set_value(g_positionSpin[ax], 0.0);
	}; 
	mov(3); //needle Z
	move_automatic(); 
	mov(5); //brake Z
	move_automatic(); 
	mov(1); //cartridge Y
	move_automatic(); 
	mov(0); // needle rot
	mov(2); // cartridge X
	mov(4); // brake rot
	mov(6); // gantry X
	mov(7); // gantry Y
	mov(8); // gantry Z
	move_automatic(); 
}

/********************************* Functions **********************************/
 
/* Description: This function prints the supported modes */
void list_capabilities(int cap)
{
    printf("Supported modes: ");
    if((cap&CAP1284_RAW))    printf("RAW ");
    if((cap&CAP1284_NIBBLE)) printf("NIBBLE ");
    if((cap&CAP1284_BYTE))   printf("BYTE ");
    if((cap&CAP1284_COMPAT)) printf("COMPAT ");
    if((cap&CAP1284_ECP))    printf("ECP ");
    if((cap&CAP1284_ECPRLE)) printf("ECPRLE ");
    if((cap&CAP1284_ECPSWE)) printf("ECPSWE ");
    if((cap&CAP1284_BECP))   printf("BECP ");
    if((cap&CAP1284_EPP))    printf("EPP ");
    if((cap&CAP1284_EPPSWE)) printf("EPPSWE ");
    if((cap&CAP1284_IRQ))    printf("IRQ ");
    if((cap&CAP1284_DMA))    printf("DMA ");
 
    printf("\n");
    return;
}

static void savePrefs(){
	MatStor ms("prefs.mat"); 
	for(int i=NAX-1; i>=0; i--){
		for(int j=1; j>=0; j--){
			ms.setValue2(i, j, "limit", (float)g_limit[i][j]); 
		}
	}
	ms.save();
}

static gboolean delete_event( GtkWidget *, GdkEvent *, gpointer ){
	g_die = true; 
	savePrefs(); 
	sleep(1); 
	gtk_main_quit ();
	return FALSE;
}

static gboolean zero_event( GtkWidget *, GdkEvent *, gpointer p){
	long i = (long)p; 
	if(i >= 0 && i < NAX){
		GtkDialogFlags flags = GTK_DIALOG_DESTROY_WITH_PARENT;
		GtkWidget* dialog = gtk_message_dialog_new (GTK_WINDOW(g_window),
								flags,
								GTK_MESSAGE_ERROR,
								GTK_BUTTONS_YES_NO,
								"Zero %s?\n(Will not move robot, \nmerely resets internal counter.)",
								g_axesLabels[i]); 
		gint r = gtk_dialog_run (GTK_DIALOG (dialog));
		if(r == GTK_RESPONSE_YES){
			g_cnt[i] = 0; 
			g_target[i] = 0; 
			update_gui(); 
		}
		gtk_widget_destroy (dialog);
	}
	return TRUE; 
}

static void positionSpinCB(GtkWidget*, gpointer p){
	if(!g_reenter){
		int h = (int)((long long)p & 0xf);
		if(h >= 0 && h < NAX){
			float a = gtk_adjustment_get_value(g_positionSpin[h]);
			setup_move(h, a, 0); 
			move_automatic(); 
			/*if(h == 4){
				while(1){
					setup_move(h, -1*a, 0);
					setup_move(2, 0, 0); 
					move_automatic();
					setup_move(h, 1*a, 0); 
					setup_move(2, -65000, 0); 
					move_automatic(); 
				}
			}*/
			if(h == 301){
				while(1){
					setup_move(h, 0, 0);
					move_automatic();
					setup_move(h, a, 0); 
					move_automatic(); 
				}
			}
		}
	}
}

static void limitEntryCB(GtkWidget*, gpointer p){
	int h = (int)((long long)p & 0xff);
	int i = h / 2; 
	int j = h % 2; 
	if(i < NAX && i >= 0 && j < 2 && j >= 0){
		const char* text = gtk_entry_get_text(GTK_ENTRY(g_limitEntry[i][j])); 
		int val = strtol(text, NULL, 10); 
		stringstream oss; 
		oss << val; 
		if(!(strcmp(oss.str().c_str(), text))) g_limit[i][j] = val; 
		else printf("ignoring: %s.\n", text); 
	}
}

static GtkAdjustment* mk_spinner(const char* txt, 
							GtkWidget* grid, int col, int row,
							 float start, float min, float max, float step,
							 GtkCallback cb, int cb_val){
	// note: takes up two columns!
	GtkWidget *spinner, *label;
	GtkAdjustment *adj;

	label = gtk_label_new (txt);
	gtk_grid_attach(GTK_GRID(grid), label, col, row, 1, 1); 
	gtk_widget_show(label);
	adj = (GtkAdjustment *)gtk_adjustment_new(
		start, min, max, step, step, 0.0);
	float climb = 1.0; int digits = 0;
	if(step <= 0.001){ climb = 0.0001; digits = 4; }
	else if(step <= 0.01){ climb = 0.001; digits = 3; }
	else if(step <= 0.1){ climb = 0.01; digits = 2; }
	else if(step <= 0.99){ climb = 0.1; digits = 1; }
	spinner = gtk_spin_button_new (adj, climb, digits);
	gtk_spin_button_set_wrap (GTK_SPIN_BUTTON (spinner), FALSE);
	g_signal_connect(spinner, "value-changed", G_CALLBACK(cb), GINT_TO_POINTER (cb_val));
	gtk_widget_show(spinner);

	//gtk_box_pack_start (GTK_BOX (container), bx, TRUE, TRUE, 2);
	gtk_grid_attach(GTK_GRID(grid), GTK_WIDGET(spinner), col+1, row, 1, 1); 
	return adj;
}

#define CMD_SIZ 5120
static char g_cmdt[CMD_SIZ]; 
static char g_stat[CMD_SIZ]; 
static char g_post[CMD_SIZ]; 

static void send_position(){
	snprintf(g_post, CMD_SIZ, "%d %d %d %d %d %d %d\n", 
				g_cnt[0], g_cnt[3], g_cnt[4], g_cnt[5],
				g_cnt[6], g_cnt[7], g_cnt[8] );
				
	int n = sendto(g_positionSock,g_post,min((int)strlen(g_post), CMD_SIZ-1), 0, 
		(struct sockaddr*)&g_positionAddr, sizeof(g_positionAddr));
	if (n < 0)
		g_connected = false; 
	else
		g_connected = true; 
}

static gboolean serviceSock(gpointer ){
	// this is called at 100 Hz. 
	bzero(g_cmdt, sizeof(g_cmdt)); 
	int n = recvfrom(g_controlSock, g_cmdt, CMD_SIZ,0,0,0); 
	if( n > 0 ){
		unsigned int i = 0; 
		char cmd[CMD_SIZ]; 
		while(i<sizeof(g_cmdt) && g_cmdt[i]){
			if(g_cmdt[i] >= 'A' && g_cmdt[i] <= 'z'){
				printf("command from controller %s ... parsing\n",g_cmdt);
				bool valid = true; 
				memcpy(cmd, &g_cmdt[i], min(strlen(&g_cmdt[i]),(size_t)(CMD_SIZ-i-1))); 
				// need to copy it - strtok modifies the string in-place. 
				char* pch = strtok(&(cmd[i]), " ");
				if( pch[0] == 'm'){
					for(int k=0; k<7; k++){
						pch = strtok(0, " "); 
						int x = atoi(pch); 
						pch = strtok(0, " "); 
						int v = atoi(pch); 
						int ax = k; 
						if(ax > 0) ax += 2; // skip cartridge axes (not used atm)
						if(v != 0){
							valid &= setup_move2(ax, x, v); 
						}
					}
					if(valid)
						move_automatic(); // and wait.
					else{
						printf(" invalid move specificed!\n");
						// clear move commands.
						for(int k=0; k<9; k++){
							g_target[k] = g_cnt[k]; 
							g_x[k] = 0.0;
							g_v[k] = 0.0;
							g_a[k] = 0.0; 
						}
					}
					//send the position before saying 'done'. 
					send_position(); 
					snprintf(g_stat, CMD_SIZ, "move done \n"); 
				}else{
					snprintf(g_stat, CMD_SIZ, "unknown command! \n"); 
				}
				if(g_statusSock > 0){
					n = sendto(g_statusSock,g_stat,min((int)strlen(g_stat), CMD_SIZ-1), 0, 
						(struct sockaddr*)&g_statusAddr, sizeof(g_statusAddr));
					if (n < 0) 
						printf("ERROR %d writing to Controller status socket\n",errno);
					else printf("move done.\n"); 
				}
			}
			update_gui(); 
			while(i < sizeof(g_cmdt) && g_cmdt[i] != '\n' && g_cmdt[i]) i++; 
			i++; //move to one past the newline. 
		}
	}
	// send the position always. 
	send_position(); 
	
	return true;
}

/* Surface to store current drawing / text */
static cairo_surface_t* g_surface = NULL;

static void clear_surface (void){
	cairo_t *cr;
	cr = cairo_create (g_surface);
	if(g_connected)
		cairo_set_source_rgb (cr, 1, 1, 1);
	else
		cairo_set_source_rgb (cr, 1, 0.8, 0.8);
	cairo_paint (cr);

	if(gtk_widget_has_focus(g_da)){
		cairo_set_line_width(cr, 5.0);
		cairo_set_source_rgb (cr, 0.7, 0.6, 1);
		cairo_move_to(cr, 1, 1);
		cairo_line_to(cr, 1, 199); 
		cairo_line_to(cr, 199, 199); 
		cairo_line_to(cr, 199, 1); 
		cairo_line_to(cr, 1, 1);
		cairo_stroke(cr);   
	}

	cairo_set_source_rgb(cr, 0.1, 0.1, 0.1); 
	cairo_select_font_face(cr, "Courier",
		CAIRO_FONT_SLANT_NORMAL, 
		CAIRO_FONT_WEIGHT_BOLD);
	
	if(g_controlAxis >=0 && g_controlAxis < 9){ 
		cairo_set_font_size(cr, 32);
		cairo_text_extents_t extents;
		cairo_text_extents(cr, g_axesLabels[g_controlAxis], &extents);
		cairo_move_to(cr, 100 - extents.width/2, 100);  
		cairo_show_text(cr, g_axesLabels[g_controlAxis]);    
		char str[256]; 
		snprintf(str, 256, "Step %d", g_controlStep[g_controlAxis]); 
		cairo_text_extents(cr, str, &extents);
		cairo_move_to(cr, 100 - extents.width/2, 100 + extents.height);  
		cairo_show_text(cr, str);    
	}
	cairo_destroy (cr);
}

/* Create a new surface of the appropriate size to store our scribbles */
static gboolean configure_event_cb (GtkWidget *widget,
                    GdkEventConfigure *, gpointer ){
	printf("configure_event\n"); 
  if (g_surface)
    cairo_surface_destroy (g_surface);
  int width, height; 
  gtk_widget_get_size_request(widget, &width, &height); 
  g_surface = gdk_window_create_similar_surface 
  		(gtk_widget_get_window (widget),
		CAIRO_CONTENT_COLOR, width, height); 
  /* Initialize the surface to white */
  clear_surface ();
  /* We've handled the configure event, no need for further processing. */
  return TRUE;
}

/* Redraw the screen from the surface. Note that the ::draw
 * signal receives a ready-to-be-used cairo_t that is already
 * clipped to only draw the exposed areas of the widget
 */
static gboolean draw_cb (GtkWidget *, cairo_t *cr, gpointer   ) {
  cairo_set_source_surface (cr, g_surface, 0, 0);
  cairo_paint (cr);
  return FALSE;
}

static gboolean keyPressCB ( GtkWidget      *widget,
							GdkEventKey *event, gpointer) { 
	/* paranoia check, in case we haven't gotten a configure event */
	if (g_surface == NULL)
		return FALSE;
	switch(event->keyval){
		case GDK_KEY_KP_1: g_controlAxis = 0; break;
		case GDK_KEY_KP_2: g_controlAxis = 1; break;
		case GDK_KEY_KP_3: g_controlAxis = 2; break;
		case GDK_KEY_KP_4: g_controlAxis = 3; break;
		case GDK_KEY_KP_5: g_controlAxis = 4; break;
		case GDK_KEY_KP_6: g_controlAxis = 5; break;
		case GDK_KEY_KP_7: g_controlAxis = 6; break;
		case GDK_KEY_KP_8: g_controlAxis = 7; break;
		case GDK_KEY_KP_9: g_controlAxis = 8; break;
		case GDK_KEY_KP_Add: g_controlStep[g_controlAxis] += 1; break;
		case GDK_KEY_KP_Subtract: g_controlStep[g_controlAxis] -= 1; break;
		case GDK_KEY_KP_Multiply: g_controlStep[g_controlAxis] *= 2; break;
		case GDK_KEY_KP_Divide: g_controlStep[g_controlAxis] /= 2; break; 
		case GDK_KEY_Up: 
			setup_move(g_controlAxis, 
						  g_cnt[g_controlAxis] + g_controlStep[g_controlAxis], 0); 
			move_automatic(); 
			update_gui(); 
			break; 
		case GDK_KEY_Down: 
			setup_move(g_controlAxis, 
						  g_cnt[g_controlAxis] - g_controlStep[g_controlAxis], 0); 
			move_automatic(); 
			update_gui(); 
			break; 
	}
	clear_surface ();
	gtk_widget_queue_draw (widget);
  /* We've handled the event, stop processing */
  return TRUE;
}
static gboolean buttonPressCB(GtkWidget      *widget,
							GdkEventKey *, gpointer) { 
	printf("button press in draw area.\n"); 
	gtk_window_set_focus(GTK_WINDOW(gtk_widget_get_toplevel(widget)), widget); 
	clear_surface ();
	gtk_widget_queue_draw (widget);
	return TRUE; 
}
static gboolean setFocusCB(GtkWidget      *widget,
							GdkEventKey *, gpointer) { 
	printf("set focus cb\n"); 
	clear_surface ();
	gtk_widget_queue_draw (widget);
	return TRUE; 
}

int main( int   argc, char *argv[]) {
	GtkWidget *button;
	GtkWidget *box1, *hbox;
	GtkWidget *grid; 
	GtkWidget *quitbox;
	
	init_globals(); 
	/* Our init, don't forget this! :) */
	gtk_init (&argc, &argv);
	
	// bidirectional socket communication. 
	g_controlSock = setup_socket(4593); 
	g_statusSock = connect_socket(4594,"localhost"); 
	get_sockaddr(4594,"localhost",&g_statusAddr);
	g_positionSock = connect_socket(4595,"localhost"); 
	get_sockaddr(4595,"localhost",&g_positionAddr); 

	/* Create our window */
	g_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);

	/* You should always remember to connect the delete_event signal
	* to the main window. This is very important for proper intuitive
	* behavior */
	g_signal_connect (g_window, "delete-event",
			G_CALLBACK (delete_event), NULL);
	gtk_container_set_border_width (GTK_CONTAINER (g_window), 10);
   
	box1 = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
	grid = gtk_grid_new(); 
	gtk_box_pack_start (GTK_BOX (box1), grid, TRUE, FALSE, 0);
	
	auto mkcolhead = [grid](const char* txt, int col){
		GtkWidget* label = gtk_label_new (txt);
		gtk_grid_attach(GTK_GRID(grid), label, col, 0, 1, 1);
		gtk_widget_show(label);
	};
	mkcolhead("Current Position", 1); 
	mkcolhead("Lower Limit", 2); 
	mkcolhead("Upper Limit", 3); 
	
	for(int i=0; i<NAX; i++){
		int step = 16; 
		if(i==0) step = 1000; 
		if(i==3) step = 10; 
		g_positionSpin[i] = mk_spinner(g_axesLabels[i], grid, 0, i+1,
			g_cnt[i], -1000000, 1000000, step, positionSpinCB, i); 
		//need upper / lower limit boxes. 
		auto mkentry = [grid, i] (int initial, int cb_val, int col) {
			GtkWidget* w = gtk_entry_new(); 
			std::stringstream oss;
			oss << initial;
			gtk_entry_set_width_chars(GTK_ENTRY(w), 8);
			//gtk_entry_set_max_length(GTK_ENTRY(w), 6); 
			gtk_entry_set_text(GTK_ENTRY(w), oss.str().c_str()); 
			gtk_widget_set_size_request(w, 50, -1);
			g_signal_connect(GTK_ENTRY(w), "changed", G_CALLBACK(limitEntryCB), GINT_TO_POINTER (cb_val));
			gtk_grid_attach(GTK_GRID(grid), w, col, i+1, 1, 1); 
			gtk_widget_show(w);
			return w; 
		}; 
		for(int j=0; j<2; j++)
			g_limitEntry[i][j] = mkentry(g_limit[i][j], i*2+j, 2+j);
		
		button = gtk_button_new_with_label("zero"); 
		g_signal_connect(button, "clicked",
				G_CALLBACK (zero_event),
				(gpointer)i);
		gtk_grid_attach(GTK_GRID(grid), button, 4, i+1, 1, 1); 
		gtk_widget_show(button);
		gtk_widget_show (grid);
	}
	
	quitbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 2);
	
	button = gtk_button_new_with_label ("Quit");
	g_signal_connect_swapped (button, "clicked",
				G_CALLBACK (delete_event),
				g_window);
	gtk_box_pack_start (GTK_BOX (quitbox), button, TRUE, FALSE, 0);
	/* pack the quitbox into the vbox (box1) */
	gtk_box_pack_start (GTK_BOX (box1), quitbox, TRUE, FALSE, 0);
	
	// make a draw widget for capturing key presses (manual movement of axes)
	hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 2);
	g_da = gtk_drawing_area_new ();
	gtk_widget_set_size_request (g_da, 200, 200);
	gtk_box_pack_start( GTK_BOX (hbox), g_da, FALSE, FALSE, 0);
	gtk_widget_set_can_focus(GTK_WIDGET(g_da), TRUE);
	gtk_widget_show (g_da);

	/* Signals used to handle the backing surface */
	g_signal_connect (g_da, "draw",
							G_CALLBACK (draw_cb), NULL);
	g_signal_connect (g_da,"configure-event",
							G_CALLBACK (configure_event_cb), NULL);

	/* Event signals */
	g_signal_connect (g_da, "key-press-event",
							G_CALLBACK (keyPressCB), NULL);
	g_signal_connect(g_da, "button-press-event", 
      					G_CALLBACK(buttonPressCB), NULL);
	g_signal_connect(g_da, "focus-out-event", 
      					G_CALLBACK(setFocusCB), NULL);
	gtk_widget_add_events(g_da, GDK_BUTTON_PRESS_MASK);
	gtk_widget_add_events(g_da, GDK_KEY_PRESS_MASK);
	gtk_widget_add_events(g_da, GDK_FOCUS_CHANGE_MASK);
	
	/* Pack the vbox (box1) which now contains all our widgets, into the
	* main window. */
	gtk_box_pack_start( GTK_BOX (hbox), box1, FALSE, FALSE, 0);
	gtk_container_add (GTK_CONTAINER (g_window), hbox);
	
	/* And show everything left */
	gtk_widget_show (button);
	gtk_widget_show (quitbox);
	
	gtk_widget_show (box1);
	gtk_widget_show (hbox);
	/* Showing the window last so everything pops up at once. */
	gtk_widget_show (g_window);
	
	parport_init(); 
	
	/*g_data = 0x0; 
	while(1){
		parport_out(); 
		usleep(1000); 
		g_data ^= 0xff000; 
	}*/
	/* version 2, bit assignment
	 * 1:  needle rot Dir		axis 0
	 * 2:  needle rot Step
	 * 3:  cartridge Y Dir		axis 1
	 * 4:  cartridge Y Step
	 * 5:  cartridge X Dir		axis 2
	 * 6:  cartridge X Step
	 * 7:  needle Z Dir			axis 3	positive = down.
	 * 8:  needle Z Step
	 * 9:  brake rot Dir			axis 4
	 * 10: brake rot Step
	 * 11: brake Z Dir			axis 5	positive = down.
	 * 12: brake Z Step
	 * */
	// add a timeout for checking the command & updating position
	g_timeout_add(16, serviceSock, NULL); 
	
	/* And of course, our main function. */
	gtk_main ();

	/* Control returns here when gtk_main_quit() is called, but not when 
	* exit() is used. */
	parport_release(); 
	
	close_socket(g_controlSock); 
	return 0;
}