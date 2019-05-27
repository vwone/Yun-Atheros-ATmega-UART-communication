#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include<stdlib.h> //exit(0);
#include<arpa/inet.h>
#include<sys/socket.h>
#include <limits.h>
#include <time.h>

//*** DEBUG ***
#define DEBUG

char InputBuffer_Tmp[255];
char InputBuffer[255];

#define FRAME_BYTES 12
int Frame_Bytes =0;

int Received_Bytes;
int Copied_Bytes;
int n;


int Err;
int Loop = 1;
#define MAX_LOOP 500

long int Pos_Int;
long int Spd_Int;
long int Tmp_Int;

long int *Pt_Pos_Int;
long int *Pt_Spd_Int;
long int *Pt_Tmp_Int;

unsigned char Pos_Int_TabChar[5];
unsigned char Spd_Int_TabChar[5];
unsigned char Tmp_Int_TabChar[5];

int init_keyboard (int fd);
int backup_keyboard (int fd);

void die(char*s)
{
    perror(s);
    exit(1);
}

void SetSpeed(struct termios *config, speed_t vitesse)
{
	cfsetispeed (config, vitesse);
	cfsetospeed (config, vitesse);
}

void Print_termios_conf(struct termios *Termios_Struct);
struct termios SavConfig_Keyboard;	

int main( int argc, char *argv[])
{
	int fd;
	char c;
	int data;
		
	struct termios Termios_Struct_Sav;	
	struct termios Termios_Struct;	
	      
	/********************************************************/
#ifdef DEBUG
	printf("uC_uP_Com : Starting...\n\r");
#endif //DEBUG
	
//O_RDWR
//Opens the port for reading and writing	
//O_NOCTTY?? Processus s'execute en arriere plan sous forme de demon
//O_NOCTTY
//The port never becomes the controlling terminal of the process.
//    Si pathname correspond à un périphérique de terminal --- voir tty(4) ---, il ne deviendra pas le terminal contrôlant le processus même si celui-ci n'est attaché à aucun autre terminal. 
//If the named file is a terminal device, don’t make it the controlling terminal for the process. 
//O_NONBLOCK ou O_NDELAY
//    Le fichier est ouvert en mode « non bloquant ». Ni la fonction open() ni aucune autre opération ultérieure sur ce fichier ne laissera le processus appelant en attente. Pour la manipulation des FIFO (tubes nommés), voir également fifo(7). Pour une discussion sur l'effet de O_NONBLOCK conjointement aux verrouillages de fichier impératifs et aux baux de fichiers, voir fcntl(2). 
//This prevents open from blocking for a “long time” to open the file. This is only meaningful for some kinds of files, usually devices such as serial ports; when it is not meaningful, it is harmless and ignored. Often, opening a port to a modem blocks until the modem reports carrier detection; if O_NONBLOCK is specified, open will return immediately without a carrier.

//Note that the O_NONBLOCK flag is overloaded as both an I/O operating mode and a file name translation flag. This means that specifying O_NONBLOCK in open also sets nonblocking I/O mode; see Operating Modes. To open the file without blocking but do normal I/O that blocks, you must call open with O_NONBLOCK set and then call fcntl to turn the bit off. 	
//Use non-blocking I/O. On some systems this also means the RS232 DCD signal line is ignored.
// O_NONBLOCK might override VMIN & VTYPE, so read may return immediately.

//ouverture non bloquante pour basculer en mode non local???

//fd = open("/dev/ttyATH0", O_RDWR );
	//17/05/19
	
	//Don't forget to give the appropriate serial ports the right  permissions (e. g.: chmod a+rw /dev/ttyS1)!
	//Open modem device for reading and writing and not as controlling tty          
	// because we don't want our porcess to get killed if linenoise sends CTRL−C.
	// VMIN et VTYPE are ignored because the O_NONBLOCK flag is set.
	fd = open("/dev/ttyATH0", O_RDWR | O_NONBLOCK | O_NOCTTY );
	if(fd<0)
	{
		perror("open");
		return(1);
	}

	//if(init_keyboard(STDIN_FILENO) != 0) return -1;

	//save current serial port settings
	memset (&Termios_Struct_Sav, 0, sizeof Termios_Struct_Sav);
	tcgetattr(fd, &Termios_Struct_Sav);

	//Get current structure to modify it
	memset (&Termios_Struct, 0, sizeof Termios_Struct);
	tcgetattr(fd, &Termios_Struct);
	
#ifdef DEBUG
	printf("uC_uP_Com : Default startup configuration : \n\r");
	Print_termios_conf(&Termios_Struct);
#endif //DEBUG


	//Not recognized at the compilation step (on both Linino / LEDE) bus probably not standard config.
	//However the scope measures a close flow of 250 000 bps when 230 400 is configured!?
	// voir setserial()
	
	//Setting non standard baud rate:
	// ioctl(fd, TCGETS2, &ntio);
	// ntio.c_cflag&= ~CBAUD;
	// ntio.c_cflag|= BOTHER;
	// ntio.c_ispeed= speed;
	// ntio.c_ospeed= speed;
	// retval=ioctl(fd, TCSETS2, &ntio); 
	// if(retval==0)
	// 		printf("New baud configured\n");
	// else
	// 		perror("ioctl");
	
	// B230400 : 0x1003
	// xxxxxxx : 0x1000 Default on LEDE
	//#define B2500000 0010014(octal) <=> 0x100C(hexa) (ref. bits/termios.h)
	// B115200 : 0x1002
	// B57600 : 0x1001
	// B38400 : 0xf
	// B19200 : 0xe
	// B9600 : 0xd
	SetSpeed(&Termios_Struct, B115200); //Measured 113 400 bps
//	SetSpeed(&Termios_Struct, B230400); //Measured ~210 000 - 260 000 bps
//	SetSpeed(&Termios_Struct, 0x1000); //Default on LEDE 0x1000 == (octal)10 000 not defined in "bits/termios.h" ?
	
	// Mode non canonique,
	//   -L'entree est immediatement disponible
	//   -l'edition de ligne est desactivee
	//   -VMIN et VTIME determinent le comportement du read.
	//
	// Modification implicite des parametres faites par la fonction cfmakeraw()
	// Termios_Struct->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	// Termios_Struct->c_oflag &= ~OPOST;
	// Termios_Struct->c_cflag &= ~(CSIZE | PARENB);
	// Termios_Struct->c_cflag |= CS8;
	// Termios_Struct->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	cfmakeraw(&Termios_Struct);

	// Complementary options to be set.
	//c_iflag
	// IXOFF if set, request the computer to control the flow of data from the terminal.
	//       the system will send START and STOP characters to the terminal to prevent loss of input data.
	Termios_Struct.c_iflag |= (IGNBRK);
	Termios_Struct.c_iflag |= (IGNPAR);
	Termios_Struct.c_iflag &= ~(INPCK);
	Termios_Struct.c_iflag &= ~(ISTRIP);
	Termios_Struct.c_iflag &= ~(INLCR);
	Termios_Struct.c_iflag &= ~(IGNCR);
	Termios_Struct.c_iflag &= ~(ICRNL);
	Termios_Struct.c_iflag &= ~(IUCLC);
	Termios_Struct.c_iflag &= ~(IXON);
	Termios_Struct.c_iflag &= ~(IXANY);
	Termios_Struct.c_iflag &= ~(IXOFF);
	Termios_Struct.c_iflag &= ~(IMAXBEL);
	Termios_Struct.c_iflag &= ~(IUTF8);
	
	//c_oflag
	// OPOST if this bit is set, it causes system-specific output data to be processed
	//       otherwise the data is transmitted without change.
	Termios_Struct.c_oflag &= ~OPOST;

	//c_cflag
	// CLOCAL = 1 => Ignore modem status lines, Ignore CD (Carrier Detect).
	// CREAD Enable receiver. If this bit is not set, no characters are received.
	// CTS/RTS Ignored 
	Termios_Struct.c_cflag |= CLOCAL; 
	Termios_Struct.c_cflag |= CREAD;
	Termios_Struct.c_cflag &= ~CRTSCTS;
	Termios_Struct.c_cflag &= ~CSTOPB;
	Termios_Struct.c_cflag &= ~HUPCL;
	Termios_Struct.c_cflag &= ~PARENB;
	//Termios_Struct.c_cflag &= ~LOBLK;
	
	Termios_Struct.c_cflag &= ~CSIZE;	// Fonctionne de paire Effacement du champs
	Termios_Struct.c_cflag |= CS8;		// Puis selection du format 5,6,7 ou 8 bits

	//c_lflag
	Termios_Struct.c_lflag = 0;
	
	//c_cc
	// In non−canonical input processing mode,
	// input is not assembled into lines and input processing (erase, kill,delete, etc.) does not occur.
	// Two parameters control the behavior of this mode:
	// c_cc[VTIME] sets thecharacter timer, and 
	// c_cc[VMIN] sets the minimum number of characters to receive before satisfying theread.
	//
	//VMIN sets the number of characters to receive before the read is satisfied.
	//TIME serves as a timeout value.
	//in tenth of seconds [0:255] => [0;0,1;...25,5] [s]
	//	If MIN > 0 and TIME = 0, MIN sets the number of characters to receive before the read is satisfied. 
	//				As TIME is zero, the timer is not used. 
	//  If MIN = 0 and TIME > 0, TIME serves as a timeout value. The read  will be satisfied if a single character isread, 
	//             or TIME is exceeded (t =  TIME *0.1 s). 
	//             If TIME is exceeded, no character will be returned. 
	// If MIN > 0 and TIME > 0, TIME serves as an inter−character timer. 
	//             The read will be satisfied if MIN characters are received, 
	//             or the time between two characters exceeds TIME. 
	//             The timer is restarted every time acharacter is received and only becomes active after the first character has been received. 
	// If MIN = 0 and TIME = 0, read will be satisfied immediately. 
	//             The  number of characters currently available,or the number of characters requested will be returned. 
	//             According to Antonino (see contributions), you could issue a fcntl(fd, F_SETFL, FNDELAY); before reading to get the same result. 
	//Termios_Struct.c_cc[VMIN] = 4;
	//Termios_Struct.c_cc[VTIME] = 1; 

#ifdef DEBUG
	printf("uC_uP_Com : New configuration : \n\r");
	Print_termios_conf(&Termios_Struct);
#endif //DEBUG
	
	//now clean the modem line 
	tcflush(fd, TCIOFLUSH);

	//and activate the settings for the port
	// Modification immediate de la configuration
	tcsetattr(fd, TCSANOW, &Termios_Struct);

	//now clean the modem line 
	tcdrain(fd);
	tcflush(fd, TCIOFLUSH);
	
	
#ifdef DEBUG
	printf("uC_uP_Com : Reseting uC...\n\r");
#endif //DEBUG

	//resetting counter on the Arduino Side
	write(fd, "RST\n", 4); 

	//waits until all output written to the object referred to by fd has been transmitted.
	//This function waits ONLY for data that has already been written with the "write()" function.
	//If you are using the standard I/O library (fprintf(), putc(), etc...) 
	//=>you must first use fflush() function ti transmit buffered data. 
	tcdrain(fd);
//	usleep(1000);//=> Acquitement du reset 0x00000000
	usleep(2000); // at 115200
//****
	do
	{
		//Read
		// O_NONBLOCK option set => read may exit imediately!
		Frame_Bytes = 0;
		do
		{
			Received_Bytes = read(fd, (unsigned char *)&InputBuffer_Tmp[0], FRAME_BYTES);
			
			if(Received_Bytes == 0)
			{
				//do nothing
			}
			
			if(Received_Bytes < 0)
			{
				#ifdef DEBUG
					sleep(2);
					printf("uC_uP_Com : Read Error (Loop %d)!!!\n\r", Loop);
				#endif //DEBUG
				
				//now clean the modem line 
				tcflush(fd, TCIOFLUSH);				
				//resetting counter on the Arduino Side
				Frame_Bytes = 0;
				write(fd, "RST\n", 4); 
				tcdrain(fd);
				//usleep(1000);//=> Acquitement du reset 0x00000000
				usleep(2000); // at 115200
				continue;
				
			}
			
			if(Received_Bytes > 0)
			{
				for( Copied_Bytes = 0 ; Copied_Bytes != Received_Bytes; Copied_Bytes++, Frame_Bytes++)
				{
					InputBuffer[Frame_Bytes] = InputBuffer_Tmp[Copied_Bytes];					
				}
			}
			
		}while(Frame_Bytes != FRAME_BYTES);
		
		Pt_Pos_Int = (long int *) &InputBuffer[0];
		Pt_Spd_Int = (long int *) &InputBuffer[4];
		Pt_Tmp_Int = (long int *) &InputBuffer[8];
		
		if(	(*Pt_Pos_Int != 0) ||
			(*Pt_Spd_Int != 0) ||
			(*Pt_Tmp_Int != 0) )
		{
			#ifdef DEBUG
			printf("!!! RST Error ## Pos : 0x%x, Spd : 0x%x, Tmp : 0x%x\n\r", Pos_Int, Spd_Int, Tmp_Int);			
			printf("%x%x%x%x, %x%x%x%x, %x%x%x%x\n\r", 
				InputBuffer[0], 
				InputBuffer[1], 
				InputBuffer[2], 
				InputBuffer[3], 
				InputBuffer[4], 
				InputBuffer[5], 
				InputBuffer[6], 
				InputBuffer[7], 
				InputBuffer[8], 
				InputBuffer[9], 
				InputBuffer[10], 
				InputBuffer[11]);			
			#endif //DEBUG
			
			//now clean the modem line 
			tcflush(fd, TCIOFLUSH);				
			Frame_Bytes = 0;
			//resetting counter on the Arduino Side
			write(fd, "RST\n", 4); 
			tcdrain(fd);
			//usleep(1000);//=> Acquitement du reset 0x00000000	
			usleep(2000); // at 115200
			continue;
		}
		
		break;
				
	}while(1);	


//*****
	// 0x55555555
	//=> Discards	terminal input data
	tcflush(fd, TCIFLUSH);		
	
#ifdef DEBUG
	printf("uC_uP_Com : Looping...\n\r");
#endif //DEBUG
	while( Loop < MAX_LOOP )
	{
		
		//En cas d'echec de la lecture
		//discards terminal input and or output data even if it hasen't yet been sent!
		//tcflush(fd, TCIOFLUSH);
		
		//Asking for Arduino Side new counter
		write(fd, "RD\n", 3);

		//waits until all output written to the object referred to by fd has been transmitted.
		//This function waits ONLY for data that has already been written with the "write()" function.
		//If you are using the standard I/O library (fprintf(), putc(), etc...) 
		//=>you must first use fflush() function ti transmit buffered data. 
		// A essayer
		tcdrain(fd);
		
		// Write + Read delay
		//
		// Output Frame delay
		// Write "RST\n" or "RD\n"
		// 4 bytes : 32 bits
		// =>
		//   32 data bits
		// + 4 Start bits
		// + 4 Stop bits
		//------------------
		// 40 bits @ BAUDRATE
		//
		// Delay = Nbits / BAUDRATE 
		//       = 40 / 250 000 = 160 µs 
		//
		// Input Frame delay 
		// 3 Integer (32bit long) => 3 x 4 bytes = 12 bytes,
		// => 
		//   96 data bits
		// + 12 Start bits,
		// + 12 Stop bits
		//------------------
		//  120 frame bits @ BAUDRATE
		//
		// Delay = Nbits / BAUDRATE 
		//       = 120 / 250 000 = 480 µs 
		// 
		// Total delay
		// 160 µs + 480 µs = 640 µs => 1000 µs : 1 ms
		//usleep(1000);
		usleep(2000); // at 115200

		//Read
		// O_NONBLOCK option set => read may exit imediately!
		Frame_Bytes = 0;
		do
		{
			Received_Bytes = read(fd, (unsigned char *)&InputBuffer_Tmp[0], FRAME_BYTES);
			
			if(Received_Bytes == 0)
			{
				//do nothing
			}
			
			if(Received_Bytes < 0)
			{
				#ifdef DEBUG
					sleep(2);					
					printf("uC_uP_Com : Read Error (Loop %d)\n\r!!!", Loop);
										
					//printf("uC_uP_Com : Read Error (Loop %d)!!! Received:", Loop);
					//received < 0!!!
					//for( n=0;  n < Received_Bytes; n++ )
					//{
					//	printf("%.2x", InputBuffer_Tmp[n] );						
					//}
					//printf("\n\r");
						
				#endif //DEBUG
				break;
			}
			
			if(Received_Bytes > 0)
			{
				for( Copied_Bytes = 0 ; Copied_Bytes != Received_Bytes; Copied_Bytes++, Frame_Bytes++)
				{
					InputBuffer[Frame_Bytes] = InputBuffer_Tmp[Copied_Bytes];					
				}
			}
			
		}while(Frame_Bytes != FRAME_BYTES);

		//don't process data if error!
		if(Received_Bytes < 0 ) 
		{
			//now clean the modem line 
			tcflush(fd, TCIOFLUSH);
			Loop++;
			continue;
		}
		
		//Cast and read values
		Pt_Pos_Int = (long int *) &InputBuffer[0];
		Pt_Spd_Int = (long int *) &InputBuffer[4];
		Pt_Tmp_Int = (long int *) &InputBuffer[8];

		Pos_Int = *Pt_Pos_Int;
		Spd_Int = *Pt_Spd_Int;
		Tmp_Int = *Pt_Tmp_Int;
						
/*		
		// Reading answer from Arduino
		// ******* Fails by chance on LEDE !!!                    ***********
		// ******* Seems the input queue is full et never flushed ***********
 		Err = 0;
		n  = read(fd, (unsigned char*)&Pos_Int, 4);
//		n  = read(fd, &Pos_Int, 1);
		if (n < 0)Err++;
//		usleep(100);
		n += read(fd, (unsigned char*)&Spd_Int, 4);
//		n += read(fd, &Spd_Int, 1);
		if (n < 0)Err++;
//		usleep(100);
		n += read(fd, (unsigned char*)&Tmp_Int, 4);
//		n += read(fd, &Tmp_Int, 1);
		if (n < 0)Err++;
//		usleep(100);
*/

 /*
		Err = 0;
		n  = read(fd, Pos_Int_TabChar, 4);
		if (n < 0)Err++;
		n += read(fd, Spd_Int_TabChar, 4);
		if (n < 0)Err++;
		n += read(fd, Tmp_Int_TabChar, 4);
		if (n < 0)Err++;

		printf("****** Loop %d (0x%x)\n\r", Loop, Loop);
		Pos_Int_TabChar[4]='\0';
		Spd_Int_TabChar[4]='\0';
		Tmp_Int_TabChar[4]='\0';

		printf("Pos 0x%.2x%.2x%.2x%.2x\n\r", Pos_Int_TabChar[0], Pos_Int_TabChar[1], Pos_Int_TabChar[2], Pos_Int_TabChar[3]);
		printf("Spd 0x%.2x%.2x%.2x%.2x\n\r", Spd_Int_TabChar[0], Spd_Int_TabChar[1], Spd_Int_TabChar[2], Spd_Int_TabChar[3]);
		printf("Tmp 0x%.2x%.2x%.2x%.2x\n\r", Tmp_Int_TabChar[0], Tmp_Int_TabChar[1], Tmp_Int_TabChar[2], Tmp_Int_TabChar[3]);
*/		

/*		
 #ifdef DEBUG
		printf("******\n\r");
		printf("%d : %d (0x%x)\n\r", Loop, Pos_Int, Pos_Int);
		printf("%d : %d (0x%x)\n\r", Loop, Spd_Int, Spd_Int);
		printf("%d : %d (0x%x)\n\r", Loop, Tmp_Int, Tmp_Int);
		if(Err != 0) printf("!!!n = %d, read error = %d\n\r", n, Err);
#endif //DEBUG
*/

		// Switches endianness from/to big<>little
		Pos_Int = htole32(Pos_Int);
		Spd_Int = htole32(Spd_Int);
		Tmp_Int = htole32(Tmp_Int);
		
		if (	( Pos_Int == 0x5555555 )&&
				( Spd_Int == 0x5555555 )&&
				( Tmp_Int == 0x5555555 ))
		{
			#ifdef DEBUG
			printf("!!! Cmd Error Loop : %d ## Pos : 0x%x, Spd : 0x%x, Tmp : 0x%x\n\r", Loop, Pos_Int, Spd_Int, Tmp_Int);			
			#endif //DEBUG
			//now clean the modem line 
			tcflush(fd, TCIOFLUSH);
			continue;					
		}

#ifdef DEBUG
		if( (Loop < 5) || (Loop > (MAX_LOOP - 5) ) )
		{
			printf("-----\n\r");
			printf("%d : %d (0x%x)\n\r", Loop, Pos_Int, Pos_Int);
			printf("%d : %d (0x%x)\n\r", Loop, Spd_Int, Spd_Int);
			printf("%d : %d (0x%x)\n\r", Loop, Tmp_Int, Tmp_Int);			
		}
#endif //DEBUG
	
		//Testing value, Loop == Count
		if (Pos_Int != Loop)
		{
			printf("!!! Val Error Loop : %d ## Val : %d\n\r", Loop, Pos_Int);			
//			sleep(1);
		}
			
		Loop++;

	}//while(Loop == MAX_LOOP)

	printf("Number of loops reached : %d\n\r", Loop);
	printf("Restoring previous port configuration...\n\r");
	sleep(2);

	//restore previous serial port settings
	tcsetattr(fd, TCSANOW, &Termios_Struct_Sav);

//	return(0);
	

	//??? STDIN_FILENO => fd
//	backup_keyboard(STDIN_FILENO);
//	backup_keyboard(fd);
	
	printf("Closing...\n\r");
	sleep(2);
	if( close(fd)<0 )
	{
		printf("Exit fails!!!\n\r");
		sleep(2);
		perror("close");
		return(1);
	}
	
	printf("Exiting soon...By!\n\r");
	sleep(2);
	return(0);
}

int init_keyboard (int fd)
{
	struct termios Config_Keyboard;	
		
	if( tcgetattr(fd, &Config_Keyboard) != 0 ) 
	{
		return -1;
	}

	// clear struct for new manual port settings
//	bzero(&Termios_Struct, sizeof(Termios_Struct)); 	

	memcpy( &SavConfig_Keyboard, &Config_Keyboard, sizeof(struct termios));
	cfmakeraw(&Config_Keyboard);
	
	Config_Keyboard.c_cc[VMIN]=0;

	if( tcsetattr(fd, TCSANOW, &Config_Keyboard) != 0)
	{
		return -1;
	}
	
	return 0;
		
}//init_keyboard()

int backup_keyboard (int fd)
{

	tcsetattr(fd, TCSANOW, &SavConfig_Keyboard);
	
	return 0;
		
}//backup_keyboard()

void Print_termios_conf(struct termios *Termios_Struct)
{
/* 	// Verifier d'abord la valeur des masques!!!
	printf("**** Verify termios.h values\n\r");
	//Input Modes : c_iflag
	printf("BRKINT : 0x%x\n\r", BRKINT);
	printf("IGNBRK : 0x%x\n\r", IGNBRK);
	printf("IGNPAR : 0x%x\n\r", IGNPAR);
	printf("PARMRK : 0x%x\n\r", PARMRK);
	printf("INPCK : 0x%x\n\r", INPCK);
	printf("ISTRIP : 0x%x\n\r", ISTRIP);
	printf("INLCR : 0x%x\n\r", INLCR);
	printf("IGNCR : 0x%x\n\r", IGNCR);
	printf("ICRNL : 0x%x\n\r", ICRNL);
	printf("IUCLC : 0x%x\n\r", IUCLC);
	printf("IXON : 0x%x\n\r", IXON);
	printf("IXANY : 0x%x\n\r", IXANY);
	printf("IXOFF : 0x%x\n\r", IXOFF);
	printf("IMAXBEL : 0x%x\n\r", IMAXBEL);
	printf("IUTF8  : 0x%x\n\r", IUTF8 );
	
	//Output Modes : c_oflag
	printf("OLCUC : 0x%x\n\r", OLCUC);
	printf("ONLCR : 0x%x\n\r", ONLCR);
	printf("OCRNL : 0x%x\n\r", OCRNL);
	printf("ONOCR : 0x%x\n\r", ONOCR);
	printf("ONLRET : 0x%x\n\r", ONLRET);
	printf("OFILL : 0x%x\n\r", OFILL);
	printf("OFDEL : 0x%x\n\r", OFDEL);
	printf("NLDLY : 0x%x\n\r", NLDLY);
	printf("CRDLY : 0x%x\n\r", CRDLY);
	printf("TABDLY : 0x%x\n\r", TABDLY);
	printf("BSDLY : 0x%x\n\r", BSDLY);
	printf("VTDLY : 0x%x\n\r", VTDLY);
	printf("FFDLY : 0x%x\n\r", FFDLY);

	//Local Modes : c_lflag
	printf("CBAUD : 0x%x\n\r", CBAUD);
	printf("CBAUDEX : 0x%x\n\r", CBAUDEX);
	printf("CS5 : 0x%x\n\r", CSIZE );
	printf("CS5 : 0x%x\n\r", CS5 );
	printf("CS6 : 0x%x\n\r", CS6 );
	printf("CS7 : 0x%x\n\r", CS7 );
	printf("CS8 : 0x%x\n\r", CS8 );
	printf("CSTOPB : 0x%x\n\r", CSTOPB);
	printf("CREAD : 0x%x\n\r", CREAD);
	printf("PARENB : 0x%x\n\r", PARENB);
	printf("PARODD : 0x%x\n\r", PARODD);
	printf("HUPCL : 0x%x\n\r", HUPCL);
	printf("CLOCAL : 0x%x\n\r", CLOCAL);
//	printf("LOBLK : 0x%x\n\r", LOBLK);
	printf("CIBAUD : 0x%x\n\r", CIBAUD);
//	printf("CMSPAR : 0x%x\n\r", CMSPAR);
	printf("CRTSCTS : 0x%x\n\r", CRTSCTS);
 	printf("B230400 : 0x%x\n\r", B230400);
 	printf("B115200 : 0x%x\n\r", B115200);
 	printf("B57600 : 0x%x\n\r", B57600);
 	printf("B38400 : 0x%x\n\r", B38400);
 	printf("B19200 : 0x%x\n\r", B19200);
 	printf("B9600 : 0x%x\n\r", B9600);

	//Control Modes : c_cflag
	printf("ISIG : 0x%x\n\r", ISIG);
	printf("ICANON : 0x%x\n\r", ICANON);
	printf("XCASE : 0x%x\n\r", XCASE);
	printf("ECHO : 0x%x\n\r", ECHO);
	printf("ECHOE : 0x%x\n\r", ECHOE);
	printf("ECHOK : 0x%x\n\r", ECHOK);
	printf("ECHONL : 0x%x\n\r", ECHONL);
	printf("ECHOCTL : 0x%x\n\r", ECHOCTL);
	printf("ECHOPRT : 0x%x\n\r", ECHOPRT);
	printf("ECHOKE : 0x%x\n\r", ECHOKE);
//	printf("DEFECHO : 0x%x\n\r", DEFECHO);
	printf("FLUSHO : 0x%x\n\r", FLUSHO);
	printf("NOFLSH : 0x%x\n\r", NOFLSH);
	printf("TOSTOP : 0x%x\n\r", TOSTOP);
	printf("PENDIN : 0x%x\n\r", PENDIN);
	printf("IEXTEN : 0x%x\n\r", IEXTEN);	

	//Control Characters : c_cc
	printf("VINTR : 0x%x\n\r", VINTR);
	printf("VQUIT : 0x%x\n\r", VQUIT);
	printf("VERASE : 0x%x\n\r", VERASE);
	printf("VKILL : 0x%x\n\r", VKILL);
	printf("VEOF : 0x%x\n\r", VEOF);
	printf("VEOL : 0x%x\n\r", VEOL);
	printf("VEOL2 : 0x%x\n\r", VEOL2);
	printf("VSWTCH : 0x%x\n\r", VSWTCH);
	printf("VSTART : 0x%x\n\r", VSTART);
	printf("VSTOP : 0x%x\n\r", VSTOP);
	printf("VSUSP : 0x%x\n\r", VSUSP);
//	printf("VDSUSP : 0x%x\n\r", VDSUSP);
	printf("VLNEXT : 0x%x\n\r", VLNEXT);
	printf("VWERASE : 0x%x\n\r", VWERASE);
	printf("VREPRINT : 0x%x\n\r", VREPRINT);
	printf("VDISCARD : 0x%x\n\r", VDISCARD);
//	printf("VSTATUS : 0x%x\n\r", VSTATUS);
	printf("VMIN : 0x%x\n\r", VMIN);
	printf("VTIME : 0x%x\n\r", VTIME);
 */
 
	printf("**** Verify termios.h configuration\n\r");
	// c_iflag
	// BRKINT : 0x2
	// IGNBRK : 0x1
	// IGNPAR : 0x4
	// PARMRK : 0x8
	// INPCK : 0x10
	// ISTRIP : 0x20
	// INLCR : 0x40
	// IGNCR : 0x80
	// ICRNL : 0x100
	// IUCLC : 0x200
	// IXON : 0x400
	// IXANY : 0x800
	// IXOFF : 0x1000
	// IMAXBEL : 0x2000
	// IUTF8  : 0x4000
	printf("*** Termios_Struct->c_iflag : 0x%x\n\r", Termios_Struct->c_iflag & Termios_Struct->c_iflag);
	printf("BRKINT : 0x%x\n\r", (Termios_Struct->c_iflag & BRKINT) == BRKINT);
	printf("IGNBRK : 0x%x\n\r", (Termios_Struct->c_iflag & IGNBRK) == IGNBRK);
	printf("IGNPAR : 0x%x\n\r", (Termios_Struct->c_iflag & IGNPAR) == IGNPAR);
	printf("PARMRK : 0x%x\n\r", (Termios_Struct->c_iflag & PARMRK) == PARMRK);
	printf("INPCK : 0x%x\n\r", (Termios_Struct->c_iflag & INPCK) == INPCK);
	printf("ISTRIP : 0x%x\n\r", (Termios_Struct->c_iflag & ISTRIP) == ISTRIP);
	printf("INLCR : 0x%x\n\r", (Termios_Struct->c_iflag & INLCR) == INLCR);
	printf("IGNCR : 0x%x\n\r", (Termios_Struct->c_iflag & IGNCR) == IGNCR);
	printf("ICRNL : 0x%x\n\r", (Termios_Struct->c_iflag & ICRNL)== ICRNL);
	printf("IUCLC : 0x%x\n\r", (Termios_Struct->c_iflag & IUCLC)== IUCLC);
	printf("IXON : 0x%x\n\r", (Termios_Struct->c_iflag & IXON)== IXON);
	printf("IXANY : 0x%x\n\r", (Termios_Struct->c_iflag & IXANY)== IXANY);
	printf("IXOFF : 0x%x\n\r", (Termios_Struct->c_iflag & IXOFF)== IXOFF);
	printf("IMAXBEL : 0x%x\n\r", (Termios_Struct->c_iflag & IMAXBEL)== IMAXBEL);
	printf("IUTF8  : 0x%x\n\r", (Termios_Struct->c_iflag & IUTF8) == IUTF8);
	
	// c_oflag
	// OPOST : 0x1
	// OLCUC : 0x2
	// ONLCR : 0x4
	// OCRNL : 0x8
	// ONOCR : 0x10
	// ONLRET : 0x20
	// OFILL : 0x40
	// OFDEL : 0x80
	// NLDLY : 0x100
	// CRDLY : 0x600
	// TABDLY : 0x1800
	// BSDLY : 0x2000
	// VTDLY : 0x4000
	// FFDLY : 0x8000
	printf("*** Termios_Struct->c_oflag : 0x%x\n\r", Termios_Struct->c_oflag);
	printf("OPOST : 0x%x\n\r", (Termios_Struct->c_oflag & OPOST) == OPOST);
	printf("OLCUC : 0x%x\n\r", (Termios_Struct->c_oflag & OLCUC) == OLCUC);
	printf("ONLCR : 0x%x\n\r", (Termios_Struct->c_oflag & ONLCR) == ONLCR);
	printf("OCRNL : 0x%x\n\r", (Termios_Struct->c_oflag & OCRNL) == OCRNL);
	printf("ONOCR : 0x%x\n\r", (Termios_Struct->c_oflag & ONOCR) == ONOCR);
	printf("ONLRET : 0x%x\n\r", (Termios_Struct->c_oflag & ONLRET) == ONLRET);
	printf("OFILL : 0x%x\n\r", (Termios_Struct->c_oflag & OFILL) == OFILL);
	printf("OFDEL : 0x%x\n\r", (Termios_Struct->c_oflag & OFDEL) == OFDEL);
	printf("NLDLY : 0x%x\n\r", (Termios_Struct->c_oflag & NLDLY) == NLDLY);
	printf("CRDLY : 0x%x\n\r", (Termios_Struct->c_oflag & CRDLY) == CRDLY);
	printf("TABDLY : 0x%x\n\r", (Termios_Struct->c_oflag & TABDLY) == TABDLY);
	printf("BSDLY : 0x%x\n\r", (Termios_Struct->c_oflag & BSDLY) == BSDLY);
	printf("VTDLY : 0x%x\n\r", (Termios_Struct->c_oflag & VTDLY) == VTDLY);
	printf("FFDLY : 0x%x\n\r", (Termios_Struct->c_oflag & FFDLY) == FFDLY);
	

	// c_cflag
	// B230400 : 0x1003
	// B115200 : 0x1002
	// B57600 : 0x1001
	// B38400 : 0xf
	// B19200 : 0xe
	// B9600 : 0xd
	// CBAUD :   0x100f
	// CIBAUD : 0x100f0000 (CIBAUD= CBAUD << IBSHIFT)
	// CBAUDEX : 0x1000
	// CSIZE : 0x30
	// CS5 : 0x0
	// CS6 : 0x10
	// CS7 : 0x20
	// CS8 : 0x30
	// CSTOPB : 0x40
	// CREAD : 0x80
	// PARENB : 0x100
	// PARODD : 0x200
	// HUPCL : 0x400
	// CLOCAL : 0x800
	// CRTSCTS : 0x80000000
	printf("*** Termios_Struct->c_cflag : 0x%x\n\r", Termios_Struct->c_cflag);

	printf("CS5 : 0x%x\n\r", (Termios_Struct->c_cflag & CSIZE) == CS5 );
	printf("CS6 : 0x%x\n\r", (Termios_Struct->c_cflag & CSIZE) == CS6 );
	printf("CS7 : 0x%x\n\r", (Termios_Struct->c_cflag & CSIZE) == CS7 );
	printf("CS8 : 0x%x\n\r", (Termios_Struct->c_cflag & CSIZE) == CS8 );

	printf("termios.h Speed defined\n\r");
 	printf("B230400 : 0x%x\n\r", B230400);
 	printf("B115200 : 0x%x\n\r", B115200);
 	printf("B57600 : 0x%x\n\r", B57600);
 	printf("B38400 : 0x%x\n\r", B38400);
 	printf("B19200 : 0x%x\n\r", B19200);
 	printf("B9600 : 0x%x\n\r", B9600);
	
	printf("Output speed\n\r");
	printf("B230400 : 0x%x\n\r", (Termios_Struct->c_cflag & CBAUD) == B230400 );
	printf("B115200 : 0x%x\n\r", (Termios_Struct->c_cflag & CBAUD) == B115200 );
	printf("B57600 : 0x%x\n\r", (Termios_Struct->c_cflag & CBAUD) == B57600 );
	printf("B38400 : 0x%x\n\r", (Termios_Struct->c_cflag & CBAUD) == B38400 );
	printf("cfgetospeed : 0x%x\n\r", cfgetospeed(Termios_Struct) );

	printf("Input speed\n\r");
	printf("B230400 : 0x%x\n\r", ((Termios_Struct->c_cflag & CIBAUD)>>16) == B230400 );
	printf("B115200 : 0x%x\n\r", ((Termios_Struct->c_cflag & CIBAUD)>>16) == B115200 );
	printf("B57600 : 0x%x\n\r", ((Termios_Struct->c_cflag & CIBAUD)>>16) == B57600 );
	printf("B38400 : 0x%x\n\r", ((Termios_Struct->c_cflag & CIBAUD)>>16) == B38400 );
	printf("cfgetispeed : 0x%x\n\r", cfgetispeed(Termios_Struct) );
	
	printf("CSTOPB : 0x%x\n\r", (Termios_Struct->c_cflag & CSTOPB) == CSTOPB);
	printf("CREAD : 0x%x\n\r", (Termios_Struct->c_cflag & CREAD) == CREAD);
	printf("PARENB : 0x%x\n\r", (Termios_Struct->c_cflag & PARENB) == PARENB);
	printf("PARODD : 0x%x\n\r", (Termios_Struct->c_cflag & PARODD) == PARODD);
	printf("HUPCL : 0x%x\n\r", (Termios_Struct->c_cflag & HUPCL) == HUPCL);
	printf("CLOCAL : 0x%x\n\r", (Termios_Struct->c_cflag & CLOCAL) == CLOCAL);
//	printf("LOBLK : 0x%x\n\r", (Termios_Struct->c_cflag & LOBLK) == LOBLK);
	printf("CIBAUD : 0x%x\n\r", (Termios_Struct->c_cflag & CIBAUD) == CIBAUD);
//	printf("CMSPAR : 0x%x\n\r", (Termios_Struct->c_cflag & CMSPAR) == CMSPAR);
	printf("CRTSCTS : 0x%x\n\r", (Termios_Struct->c_cflag & CRTSCTS) == CRTSCTS);


	// c_lflag
	// ISIG : 0x1
	// ICANON : 0x2
	// XCASE : 0x4
	// ECHO : 0x8
	// ECHOE : 0x10
	// ECHOK : 0x20
	// ECHONL : 0x40
	// ECHOCTL : 0x200
	// ECHOPRT : 0x400
	// ECHOKE : 0x800
	// FLUSHO : 0x2000
	// NOFLSH : 0x80
	// TOSTOP : 0x8000
	// PENDIN : 0x4000
	// IEXTEN : 0x100
	printf("*** Termios_Struct->c_lflag : : 0x%x\n\r", Termios_Struct->c_lflag);
	printf("ISIG : 0x%x\n\r", (Termios_Struct->c_lflag & ISIG) == ISIG);
	printf("ICANON : 0x%x\n\r", (Termios_Struct->c_lflag & ICANON) == ICANON);
	printf("XCASE : 0x%x\n\r", (Termios_Struct->c_lflag & XCASE) == XCASE);
	printf("ECHO : 0x%x\n\r", (Termios_Struct->c_lflag & ECHO) == ECHO);
	printf("ECHOE : 0x%x\n\r", (Termios_Struct->c_lflag & ECHOE) == ECHOE);
	printf("ECHOK : 0x%x\n\r", (Termios_Struct->c_lflag & ECHOK) == ECHOK);
	printf("ECHONL : 0x%x\n\r", (Termios_Struct->c_lflag & ECHONL) == ECHONL);
	printf("ECHOCTL : 0x%x\n\r", (Termios_Struct->c_lflag & ECHOCTL) == ECHOCTL);
	printf("ECHOPRT : 0x%x\n\r", (Termios_Struct->c_lflag & ECHOPRT) == ECHOPRT);
	printf("ECHOKE : 0x%x\n\r", (Termios_Struct->c_lflag & ECHOKE) == ECHOKE);
//	printf("DEFECHO : 0x%x\n\r", (Termios_Struct->c_lflag & DEFECHO) == DEFECHO);
	printf("FLUSHO : 0x%x\n\r", (Termios_Struct->c_lflag & FLUSHO) == FLUSHO);
	printf("NOFLSH : 0x%x\n\r", (Termios_Struct->c_lflag & NOFLSH) == NOFLSH);
	printf("TOSTOP : 0x%x\n\r", (Termios_Struct->c_lflag & TOSTOP) == TOSTOP);
	printf("PENDIN : 0x%x\n\r", (Termios_Struct->c_lflag & PENDIN) == PENDIN);
	printf("IEXTEN : 0x%x\n\r", (Termios_Struct->c_lflag & IEXTEN) == IEXTEN);	

	// c_cc
	// VINTR : 0x0
	// VQUIT : 0x1
	// VERASE : 0x2
	// VKILL : 0x3
	// VEOF : 0x10
	// VEOL : 0x11
	// VEOL2 : 0x6
	// VSWTCH : 0x7
	// VSTART : 0x8
	// VSTOP : 0x9
	// VSUSP : 0xa
	// VLNEXT : 0xf
	// VWERASE : 0xe
	// VREPRINT : 0xc
	// VDISCARD : 0xd
	// VMIN : 0x4
	// VTIME : 0x5
	printf("*** Termios_Struct->c_cc\n\r");
	printf("VINTR : 0x%x\n\r", Termios_Struct->c_cc[ VINTR ]);
	printf("VQUIT : 0x%x\n\r", Termios_Struct->c_cc[ VQUIT ]);
	printf("VERASE : 0x%x\n\r", Termios_Struct->c_cc[ VERASE ]);
	printf("VKILL : 0x%x\n\r", Termios_Struct->c_cc[ VKILL ]);
	printf("VEOF : 0x%x\n\r", Termios_Struct->c_cc[ VEOF ]);
	printf("VEOL : 0x%x\n\r", Termios_Struct->c_cc[ VEOL ]);
	printf("VEOL2 : 0x%x\n\r", Termios_Struct->c_cc[ VEOL2 ]);
	printf("VSWTCH : 0x%x\n\r", Termios_Struct->c_cc[ VSWTCH ]);
	printf("VSTART : 0x%x\n\r", Termios_Struct->c_cc[ VSTART ]);
	printf("VSTOP : 0x%x\n\r", Termios_Struct->c_cc[ VSTOP ]);
	printf("VSUSP : 0x%x\n\r", Termios_Struct->c_cc[ VSUSP ]);
//	printf("VDSUSP : 0x%x\n\r", Termios_Struct->c_cc[ VDSUSP ]);
	printf("VLNEXT : 0x%x\n\r", Termios_Struct->c_cc[ VLNEXT ]);
	printf("VWERASE : 0x%x\n\r", Termios_Struct->c_cc[ VWERASE ]);
	printf("VREPRINT : 0x%x\n\r", Termios_Struct->c_cc[ VREPRINT ]);
	printf("VDISCARD : 0x%x\n\r", Termios_Struct->c_cc[ VDISCARD ]);
//	printf("VSTATUS : 0x%x\n\r", Termios_Struct->c_cc[ VSTATUS ]);
	printf("VMIN : 0x%x\n\r", Termios_Struct->c_cc[ VMIN ]);
	printf("VTIME : 0x%x\n\r", Termios_Struct->c_cc[ VTIME ]);
	
	return;
	
}//Print_termios_conf