#ifndef HRSCRIPT_H
#define HRSCRIPT_H

#include "stdio.h"
#include "WPILib.h"
#include <string>


const int MAX_HRSCRIPT_COMMAND_PARAMS = 16;
const int MAX_HRSCRIPT_COMMANDS = 256;
const int MAX_AUTO_MODES = 64;
const int MAX_COMMAND_PROTOTYPES = 256;


class HrScriptCommandClass
{
public:
	HrScriptCommandClass();
	virtual ~HrScriptCommandClass();

	// User implements these so the system can create your command as it parses a script
	virtual const char * Get_Command_Name() = 0;
	virtual int Get_Parameter_Count() = 0;
	virtual HrScriptCommandClass * Create_Command() = 0;

	// Implement the Execute function which will run a script executes your command
	virtual void Execute() = 0;

	// access to the parameters
	void Set_Parameter(int i,float val);
	float Get_Parameter(int i);

protected:

	float m_Parameters[MAX_HRSCRIPT_COMMAND_PARAMS];
};




class HrScriptClass
{
public:
	HrScriptClass(void);
	~HrScriptClass(void);

	void Add_Command(HrScriptCommandClass * cmd);
	int Get_Command_Count() { return m_CommandCount; }
	HrScriptCommandClass * Get_Command(int i);

	void Set_Name(const char * name) { m_Name = name; }
	const char * Get_Name(void) { return m_Name.c_str(); }

protected:

	std::string m_Name;
	int m_CommandCount;
	HrScriptCommandClass * m_Commands[MAX_HRSCRIPT_COMMANDS];
};


class HrScriptSystemClass
{
public:
	HrScriptSystemClass();
	~HrScriptSystemClass();

	// Register all of your robots commands
	void Add_Command(HrScriptCommandClass * command);

	// Load and release Scripts
	HrScriptClass * Load_Script(const char * filename);
	void Release_Script(HrScriptClass *);

	// Run a script
	void Execute_Script(HrScriptClass * script);

	// Autonomous Scripts
	void Set_Auto_Script(int index,const char * script_filename);
	void Run_Auto_Script(int index);

	// Helper function, loads, runs, releases a script
	void Load_And_Run_Script(const char * filename);

protected:

	HrScriptCommandClass * Load_Command(std::string cmd_string);

	// One of each command the user's robot has registered.
	int m_CommandPrototypeCount;
	HrScriptCommandClass * m_CommandPrototypes[MAX_COMMAND_PROTOTYPES];

	// Various Autonomous mode scripts
	HrScriptClass * m_Scripts[MAX_AUTO_MODES];

};

#endif
