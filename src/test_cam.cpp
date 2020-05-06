#include "ros/ros.h"
#include "genie_nano/cordef.h"
#include "GenApi/GenApi.h"					//!< GenApi lib definitions.
#include "genie_nano/gevapi.h"				//!< GEV lib definitions.
#include "genie_nano/SapX11Util.h"
#include "genie_nano/X_Display_utils.h"
#include "genie_nano/FileUtil.h"
#include <sched.h>

#include <iostream>

#define LOOP_TIME   			0.5

#define MAX_NETIF				8
#define MAX_CAMERAS_PER_NETIF	32
#define MAX_CAMERAS				(MAX_NETIF * MAX_CAMERAS_PER_NETIF)

#define NUM_BUF					8

#define ENABLE_BAYER_CONVERSION 1


using namespace std;

typedef struct tagMY_CONTEXT
{
	X_VIEW_HANDLE		View;
	GEV_CAMERA_HANDLE	camHandle;
	int					depth;
	int 				format;
	void 				*convertBuffer;
	BOOL				convertFormat;
	BOOL				exit;
}MY_CONTEXT, *PMY_CONTEXT;

// ---------------------- //
// -- Grobal Variables -- //
// ---------------------- //

void *m_latestBuffer = NULL;

// ----------------------- //
// -- General Functions -- //
// ----------------------- //

char GetKey();
void * ImageDisplayThread(void *context);
static unsigned long us_timer_init(void);
int IsTurboDriveAvailable(GEV_CAMERA_HANDLE handle);

// ------------------------ //
// -- Callback Functions -- //
// ------------------------ //


// ------------------- //
// -- Main Function -- //
// ------------------- //

int main (int argc, char **argv){
	// ----------------- //
	// -- ROS setting -- //
	// ----------------- //
	ros::init(argc, argv, "test_cam");
	ros::NodeHandle nh;

	//ros::Rate loop_rate(1/LOOP_TIME);
	ros::Rate loop_rate(0.5);

	double count = 0;
	char c;

	// -------------------- //
	// -- Camera setting -- //
	// -------------------- //
	GEV_DEVICE_INTERFACE pCamera[MAX_CAMERAS] = {0};
	GEV_STATUS status;
	int numCamera = 0;
	int camIndex = 0;
	X_VIEW_HANDLE  View = NULL;
	MY_CONTEXT context = {0};
	pthread_t  tid;
	char c_key;
	int done = FALSE;
	int turboDriveAvailable = 0;
	char uniqueName[128];
	uint32_t macLow = 0; // Low 32-bits of the mac address (for file naming).

	cout << endl << "GigE Vision Library GenICam C-- Example Program" << __DATE__ << endl;
	cout << "Copyright (c) 2105, DALSA." << endl << "All rights reserved." << endl;

	// Set default options for the library.
	{
		GEVLIB_CONFIG_OPTIONS options = {0};
		GevGetLibraryConfigOptions( &options);
		//options.logLevel = GEV_LOG_LEVEL_OFF;
		//options.logLevel = GEV_LOG_LEVEL_TRACE;
		options.logLevel = GEV_LOG_LEVEL_NORMAL;
		GevSetLibraryConfigOptions( &options);
	}

	// DISCOVER Cameras
	// Get all the IP addresses of attached network cards.
	status = GevGetCameraList( pCamera, MAX_CAMERAS, &numCamera);

	cout << numCamera << "camera(s) on the network" << endl;

	if(numCamera != 0){
		if(camIndex != -1){
			// Connect to Camera
			int i;
			int type;
			UINT32 height = 0;
			UINT32 width = 0;
			UINT32 format = 0;
			UINT32 maxHeight = 1600;
			UINT32 maxWidth = 2048;
			UINT32 maxDepth = 2;
			UINT64 size;
			UINT64 payload_size;
			int numBuffers = NUM_BUF;
			PUINT8 bufAddress[NUM_BUF];
			GEV_CAMERA_HANDLE handle = NULL;
			UINT32 pixFormat = 0;
			UINT32 pixDepth = 0;
			UINT32 convertedGevFormat = 0;
			
			// Open the camera.
			status = GevOpenCamera( &pCamera[camIndex], GevExclusiveMode, &handle);
			if (status == 0){
				// GenICam feature access via Camera XML File enabled by "open"
				// Get the name of XML file name back (example only - in case you need it somewhere).
				char xmlFileName[MAX_PATH] = {0};
				status = GevGetGenICamXML_FileName( handle, (int)sizeof(xmlFileName), xmlFileName);
				if (status == GEVLIB_OK){
					cout << "XML stored as " << xmlFileName << endl;
				}
				status = GEVLIB_OK;
			}
			// Get the low part of the MAC address (use it as part of a unique file name for saving images).
			// Generate a unique base name to be used for saving image files
			// based on the last 3 octets of the MAC address.
			macLow = pCamera[camIndex].macLow;
			macLow &= 0x00FFFFFF;
			snprintf(uniqueName, sizeof(uniqueName), "img_%06x", macLow); 
			
			// Go on to adjust some API related settings (for tuning / diagnostics / etc....).
			if ( status == 0 ){
				GEV_CAMERA_OPTIONS camOptions = {0};

				// Adjust the camera interface options if desired (see the manual)
				GevGetCameraInterfaceOptions( handle, &camOptions);
				//camOptions.heartbeat_timeout_ms = 60000;		// For debugging (delay camera timeout while in debugger)
				camOptions.heartbeat_timeout_ms = 5000;		// Disconnect detection (5 seconds)
				// Write the adjusted interface options back.
				GevSetCameraInterfaceOptions( handle, &camOptions);

				// Get the GenICam FeatureNodeMap object and access the camera features.
				GenApi::CNodeMapRef *Camera = static_cast<GenApi::CNodeMapRef*>(GevGetFeatureNodeMap(handle));
				
				if(Camera){
					// Access some features using the bare GenApi interface methods
					try {
						//Mandatory features....
						GenApi::CIntegerPtr ptrIntNode = Camera->_GetNode("Width");
						width = (UINT32) ptrIntNode->GetValue();
						ptrIntNode = Camera->_GetNode("Height");
						height = (UINT32) ptrIntNode->GetValue();
						ptrIntNode = Camera->_GetNode("PayloadSize");
						payload_size = (UINT64) ptrIntNode->GetValue();
						GenApi::CEnumerationPtr ptrEnumNode = Camera->_GetNode("PixelFormat") ;
						format = (UINT32)ptrEnumNode->GetIntValue();
					}
					// Catch all possible exceptions from a node access.
					CATCH_GENAPI_ERROR(status);
				}

				if(status == 0){
					// Set up a grab/transfer from this camera
					cout << "Camera ROI set for" << endl;
					cout << "Height = " << height << endl;
					cout << "Width = " << width << endl;
					cout << "PixelFormat (val) = " << format << endl;;

					maxHeight = height;
					maxWidth = width;
					maxDepth = GetPixelSizeInBytes(format);

					// Allocate image buffers
					// (Either the image size or the payload_size, whichever is larger - allows for packed pixel formats).
					size = maxDepth * maxWidth * maxHeight;
					size = (payload_size > size) ? payload_size : size;
					for(i = 0; i < numBuffers; i++){
						bufAddress[i] = (PUINT8)malloc(size);
						memset(bufAddress[i], 0, size);
					}
					//status = GevInitializeTransfer( handle, Asynchronous, size, numBuffers, bufAddress);
					status = GevInitializeTransfer( handle, SynchronousNextEmpty, size, numBuffers, bufAddress);
					// Create an image display window.
					// This works best for monochrome and RGB. The packed color formats (with Y, U, V, etc..) require 
					// conversion as do, if desired, Bayer formats.
					// (Packed pixels are unpacked internally unless passthru mode is enabled).

					// Translate the raw pixel format to one suitable for the (limited) Linux display routines.			

					status = GetX11DisplayablePixelFormat( ENABLE_BAYER_CONVERSION, format, &convertedGevFormat, &pixFormat);

					if (format != convertedGevFormat){
						// We MAY need to convert the data on the fly to display it.
						if (GevIsPixelTypeRGB(convertedGevFormat)){
							// Conversion to RGB888 required.
							pixDepth = 32;	// Assume 4 8bit components for color display (RGBA)
							context.format = Convert_SaperaFormat_To_X11( pixFormat);
							context.depth = pixDepth;
							context.convertBuffer = malloc((maxWidth * maxHeight * ((pixDepth + 7)/8)));
							context.convertFormat = TRUE;
						}else{
							// Converted format is MONO - generally this is handled
							// internally (unpacking etc...) unless in passthru mode.
							// (						
							pixDepth = GevGetPixelDepthInBits(convertedGevFormat);
							context.format = Convert_SaperaFormat_To_X11( pixFormat);
							context.depth = pixDepth;							
							context.convertBuffer = NULL;
							context.convertFormat = FALSE;
						}
					}else{
						pixDepth = GevGetPixelDepthInBits(convertedGevFormat);
						context.format = Convert_SaperaFormat_To_X11( pixFormat);
						context.depth = pixDepth;
						context.convertBuffer = NULL;
						context.convertFormat = FALSE;
					}
					
					View = CreateDisplayWindow("GigE-V GenApi Console Demo", TRUE, height, width, pixDepth, pixFormat, FALSE ); 

					// Create a thread to receive images from the API and display them.
					context.View = View;
					context.camHandle = handle;
					context.exit = FALSE;
		   			pthread_create(&tid, NULL, ImageDisplayThread, &context);

					// See if TurboDrive is available.
					turboDriveAvailable = IsTurboDriveAvailable(handle);
					if(turboDriveAvailable){
						UINT32 val = 1;
						GevGetFeatureValue(handle, "transferTurboMode", &type, sizeof(UINT32), &val);
						val = (val == 0) ? 1 : 0;
						GevSetFeatureValue(handle, "transferTurboMode", sizeof(UINT32), &val);
						GevGetFeatureValue(handle, "transferTurboMode", &type, sizeof(UINT32), &val);
						if (val == 1){
							cout << "TurboMode Enabled" << endl;
						}else{
							cout << "TurboMode Disabled" << endl;
						}
					}else{
						cout << "*** TurboDrive is NOT Available for this device/pixel format combination ***" << endl;
					}

					for(i = 0; i < numBuffers; i++){
						memset(bufAddress[i], 0, size);
					}
					status = GevStartTransfer(handle, -1);
					if(status != 0){
						cout << "Error starting grab - " << status << endl; 
					}

					while(ros::ok()){
						
						ros::spinOnce();
						loop_rate.sleep();
						count = count + LOOP_TIME;
						cout << count << endl;
					}
					GevStopTransfer(handle);
					done = TRUE;
					context.exit = TRUE;
					pthread_join( tid, NULL);      
					GevAbortTransfer(handle);
					status = GevFreeTransfer(handle);
					DestroyDisplayWindow(View);
					for (i = 0; i < numBuffers; i++){	
						free(bufAddress[i]);
					}
					if (context.convertBuffer != NULL){
						free(context.convertBuffer);
						context.convertBuffer = NULL;
					}
				}
				GevCloseCamera(&handle);
			}else{
				cout << "Error :" << status << ": opening camera" << endl;
			}
		}
	}
	// Close down the API.
	GevApiUninitialize();

	// Close socket API
	_CloseSocketAPI ();	// must close API even on error

	return 0;
}

// ----------------------- //
// -- General Functions -- //
// ----------------------- //

char GetKey(){
   char key = getchar();
   while ((key == '\r') || (key == '\n')){
      key = getchar();
   }
   return key;
}


void * ImageDisplayThread( void *context){
	MY_CONTEXT *displayContext = (MY_CONTEXT *)context;

	if (displayContext != NULL)
	{
   	unsigned long prev_time = 0;
   	//unsigned long cur_time = 0;
		//unsigned long deltatime = 0;
		prev_time = us_timer_init();

		// While we are still running.
		while(!displayContext->exit)
		{
			GEV_BUFFER_OBJECT *img = NULL;
			GEV_STATUS status = 0;
	
			// Wait for images to be received
			status = GevWaitForNextImage(displayContext->camHandle, &img, 1000);

			if ((img != NULL) && (status == GEVLIB_OK))
			{
				if (img->status == 0)
				{
					m_latestBuffer = img->address;
					// Can the acquired buffer be displayed?
					if ( IsGevPixelTypeX11Displayable(img->format) || displayContext->convertFormat )
					{
						// Convert the image format if required.
						if (displayContext->convertFormat)
						{
							int gev_depth = GevGetPixelDepthInBits(img->format);
							// Convert the image to a displayable format.
							//(Note : Not all formats can be displayed properly at this time (planar, YUV*, 10/12 bit packed).
							ConvertGevImageToX11Format( img->w, img->h, gev_depth, img->format, img->address, \
													displayContext->depth, displayContext->format, displayContext->convertBuffer);
					
							// Display the image in the (supported) converted format. 
							Display_Image( displayContext->View, displayContext->depth, img->w, img->h, displayContext->convertBuffer );				
						}
						else
						{
							// Display the image in the (supported) received format. 
							Display_Image( displayContext->View, img->d,  img->w, img->h, img->address );
						}
					}
					else
					{
						//printf("Not displayable\n");
					}
				}
				else
				{
					// Image had an error (incomplete (timeout/overflow/lost)).
					// Do any handling of this condition necessary.
				}
			}
#if USE_SYNCHRONOUS_BUFFER_CYCLING
			if (img != NULL)
			{
				// Release the buffer back to the image transfer process.
				GevReleaseImage( displayContext->camHandle, img);
			}
#endif
		}
	}
	pthread_exit(0);	
}

static unsigned long us_timer_init( void ){
   struct timeval tm;
   unsigned long msec;
   
   // Get the time and turn it into a millisecond counter.
   gettimeofday( &tm, NULL);
   
   msec = (tm.tv_sec * 1000000) + (tm.tv_usec);
   return msec;
}

int IsTurboDriveAvailable(GEV_CAMERA_HANDLE handle){
	int type;
	UINT32 val = 0;
	
	if ( 0 == GevGetFeatureValue( handle, "transferTurboCurrentlyAbailable",  &type, sizeof(UINT32), &val) )
	{
		// Current / Standard method present - this feature indicates if TurboMode is available.
		// (Yes - it is spelled that odd way on purpose).
		return (val != 0);
	}
	else
	{
		// Legacy mode check - standard feature is not there try it manually.
		char pxlfmt_str[64] = {0};

		// Mandatory feature (always present).
		GevGetFeatureValueAsString( handle, "PixelFormat", &type, sizeof(pxlfmt_str), pxlfmt_str);

		// Set the "turbo" capability selector for this format.
		if ( 0 != GevSetFeatureValueAsString( handle, "transferTurboCapabilitySelector", pxlfmt_str) )
		{
			// Either the capability selector is not present or the pixel format is not part of the 
			// capability set.
			// Either way - TurboMode is NOT AVAILABLE.....
			return 0; 
		}
		else
		{
			// The capabilty set exists so TurboMode is AVAILABLE.
			// It is up to the camera to send TurboMode data if it can - so we let it.
			return 1;
		}
	}
	return 0;
}

// ------------------------ //
// -- Callback Functions -- //
// ------------------------ //
