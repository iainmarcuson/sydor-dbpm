import socket
import sys
import select
import os
import traceback
import time

BSHARP_ADDR = '192.168.11.91';

packet_count = 0;
CMD_LEN = 32;                  # Maximum length of command to try to filter out.  Actual max for a command is 25, but add a litle padding.

#Returns a status code and a byte array corresponding to a message
RECV_PARTIAL = 0;
RECV_FULL = 1;
RECV_FLUSH = 2;                 # If an erroneous packet is detected, signal to flush the buffer


## YF [
ALLOW_FLUSH = True
FLUSH = False
#internal_log = []    # use this to debug
internal_log = None   # use this for release
## YF ]

FIFO_DIRTY = False;
desync_packets = 0;

DEFAULT_SELECT = 0.02;
select_timeout = DEFAULT_SELECT;




def bsharp_all_recv(in_bytes):
    global FIFO_DIRTY;
    global desync_packets;
    ###if len(in_bytes) >= 3:
    ###    print("all_recv: {:2x} {:2x} {:2x} {:2x}".format(in_bytes[0], in_bytes[1], in_bytes[2], in_bytes[3]));

    #YF[
    log_len_inbytes = len(in_bytes)
    #YF]

    if in_bytes.find(b'bb') == 0: # Data
        #print("bb size:  {}".format(len(in_bytes)));
        delimit_idx = in_bytes.find(b'\x01');
        if (delimit_idx < 0):
            if internal_log != None:
                internal_log.append( str(log_len_inbytes) + ", " +  "RP, L42")

            return (RECV_PARTIAL, b'');
        data_size = in_bytes[delimit_idx+1]*256+in_bytes[delimit_idx+2];
        #print("bb payload size: {}".format(data_size));
        ###
        #debug_string = "";
        #if (len(in_bytes) > 10):
        #    for byte_idx in range(8):
        #        debug_string = debug_string+"{:02x},".format(in_bytes[byte_idx]);
        #    print("Data: {}".format(debug_string));
        #print("Data Size: {} Payload Size: {}".format(len(in_bytes), data_size));
        expected_size =     3+2+2+data_size+1+0;
            #               | | | |         | +-> Added header
            #               | | | |         +-> Checksum
            #               | | | +-----------> Payload
            #               | | +-------------> Size bytes
            #               | +---------------> "B\x01"
            #               +-----------------> "rb>"
            

        if len(in_bytes) >= expected_size:
            #print("bb returning {} of {} bytes.".format(len(in_bytes[0:expected_size]), len(in_bytes)));
            if internal_log != None:
               internal_log.append( str(log_len_inbytes) + ", " +  "RF, L70")
            return (RECV_FULL, in_bytes[0:expected_size]);
        else:
            if internal_log != None:
               internal_log.append( str(log_len_inbytes) + ", " +  "RP, L76")
            return (RECV_PARTIAL, b'');

    elif in_bytes.find(b'B\x01') == 0:
        #print("B1 size: {}".format(len(in_bytes)));
        delimit_idx = in_bytes.find(b'\x01');
        data_size = in_bytes[delimit_idx+1]*256+in_bytes[delimit_idx+2];
        #print("Data size is: {}".format(data_size));
        expected_size =     2+2+data_size+1+0;
            #               | | |         | +--> Extra header
            #               | | |         +-> Checksum
            #               | | +-----------> Payload
            #               | +-------------> Size bytes
            #               +---------------> "B\x01"
            #

        if len(in_bytes) >= expected_size:
            #print("B1 returning {} of {} bytes.".format(len(in_bytes[0:expected_size]), len(in_bytes)));
            if internal_log != None:
               internal_log.append( str(log_len_inbytes) + ", " +  "RF, L86")
            return (RECV_FULL, in_bytes[0:expected_size]);
        else:
            if internal_log != None:
               internal_log.append( str(log_len_inbytes) + ", " +  "RP, L89")
            return (RECV_PARTIAL, b'');

    else:             # Assume Command
        #if FIFO_DIRTY:
            #try:
            #    print("Received command bytes: {}".format(in_bytes[0:16].decode()));
            #except:
            #    if (len(in_bytes))>=4:
            #        print("Received command hex: {:3d} {:3d} {:3d} {:3d}".format(in_bytes[0], in_bytes[1], in_bytes[2], in_bytes[3]));
            #        foo = 123;
        #command_hex = "Command hex is: ";
        #for curr_byte in in_bytes:
        #    command_hex = command_hex + "{:2x} ".format(curr_byte);
        #print(command_hex);
        FIFO_DIRTY = False;
        # Check to see if we have a valid response.  Assume no whitespace
        # XXX Assuming no whitespace
        if (in_bytes.find(b'bs')==0) or (in_bytes.find(b'rr')==0) or (in_bytes.find(b'bc')==0) or (in_bytes.find(b'wr')==0):
            pass;               # A valid command
        else:                   # Erroneos data, so flush
            desync_packets = desync_packets+1; # Note a desynchronization
            return (RECV_FLUSH, b'');
        cmd_end = in_bytes.find(b'\r\n'); 

        if (cmd_end>0) and (cmd_end<CMD_LEN):
            return (RECV_FULL, in_bytes[0:cmd_end+2]);
        elif len(in_bytes)<CMD_LEN:
            return (RECV_PARTIAL, b'');
        else:
            return (RECV_FLUSH, b'');

def read_intelligent(in_socket):
    out_bytes = b'';
    read_list = in_socket;

    while True:
        #readable, writable,errored = select.select([read_list], [], [], 0.1); # Just the one socket, and provide some provision for timeout
        readable, writable,errored = select.select([read_list], [], [], 0.2); # Just the one socket, and provide some provision for timeout
        if readable == []:      # Nothing ready
            out_bytes = b'';
            return out_bytes;
        
        curr_bytes = in_socket.recv(1024);
        if len(curr_bytes)>=4:
            #print("Start of packet: {:3} {:3} {:3} {:3}".format(curr_bytes[0], curr_bytes[1], curr_bytes[2], curr_bytes[3]));
            pass;
        out_bytes = out_bytes + curr_bytes;
        ###
        #print("Data length: {}".format(len(out_bytes)));
        if bsharp_all_recv(out_bytes):
            return out_bytes;
        
def read_all(in_socket):
    out_bytes = b'';
    read_list = in_socket;

    while True:
        readable,writable,errored = select.select([read_list], [], [], 0.001); # Just the one socket, and don't delay
        if readable == []:       # Nothing ready
            break;
        
        curr_bytes = in_socket.recv(1024);
        out_bytes = out_bytes+curr_bytes;
        
    return out_bytes;

BSHARP_PORT = 13000;
#BSHARP_ADDR = '127.0.0.1';
#BSHARP_ADDR = '192.168.11.91';
CMD_PORT = 13001;
DATA_PORT = 13002;

cmd_connected = False;
data_connected = False;

cmd_get_time = 0.0;
cmd_finish_time = 0.0;
# Create the connection to the B#
bsharp_sock = socket.socket();
bsharp_sock.connect((BSHARP_ADDR, BSHARP_PORT));

# Create the command and data sockets
cmd_sock = socket.socket();
cmd_sock.bind(('',CMD_PORT));
cmd_sock.listen(3);

data_sock = socket.socket();
data_sock.bind(('',DATA_PORT));
data_sock.listen(3);

cmd_client_sock = None;
data_client_sock = None;

data_count = 0;

DATA_NONE = 0;                  # No data pending from BSharp
DATA_CURR = 1;                  # Current data pending from BSharp
DATA_CMD = 2;                   # Command data pending from BSharp

data_in_count = 0;
data_out_count = 0;

curr_minutes = int(time.time()/60);
old_minutes = int(time.time()/60);

bsharp_sock.send(b'bs 0 4\r\n');
go_return = bsharp_sock.recv(1024);
#print("Go command reported: {}".format(go_return.decode()));
bsharp_sock.send(b'wr 154 0\r\n');
go_return = bsharp_sock.recv(1024);
bsharp_sock.send(b'bc 152 2\r\n');
go_return = bsharp_sock.recv(1024);
### Not actually a function bsharp_sock.flush();
#go_return = bsharp_sock.recv(1024);
#print("Reg 1 is :{}".format(go_return.decode()));

# Time to start the main loop
try:
    from_bsharp_socket = b'';   # Start with an empty "FIFO"
    FIFO_DIRTY = False;         # FIFO is initially clean
    data_pending = DATA_NONE;
    ## YF[
    dump_counter = 0
    ## YF]
    while True:
        # Python won't accept None in the list, so that must be added manually
        write_list = [];
        read_list = [cmd_sock, data_sock, bsharp_sock];
        if cmd_client_sock != None:
            read_list.append(cmd_client_sock);
            write_list.append(cmd_client_sock);
        if data_client_sock != None:
            read_list.append(data_client_sock);
            #print(data_pending)
            write_list.append(data_client_sock);
            

        
        
        read_list, write_list, error_list = select.select(read_list, write_list, [], select_timeout);

        select_timeout = DEFAULT_SELECT; # Always reset to default, as sometimes we might do a fast timeout;
        
        ## YF[
        if len(read_list) == 0:
            if FLUSH:
                FLUSH = False
                bsharp_sock.send(b'bc 152 2\r\n');   # RE-START  Transmitting

        ## YF]


        curr_minutes = int(time.time()/10);  ## YF

        if (curr_minutes != old_minutes):
            old_minutes = curr_minutes;
            print("In: {}\tOut: {}".format(data_in_count, data_out_count));
            if (desync_packets > 0):
                print("Desync count: {}".format(desync_packets));
                desync_packets = 0;
            data_in_count = 0;
            data_out_count = 0;

        for curr_readable in read_list:
            #print("Readable socket. {} items".format(len(read_list)));
            if curr_readable == cmd_sock:
                print("Cmd socket readable.");
                if cmd_connected: # Already have a connection
                    temp_sock, temp_addr = cmd_sock.accept();
                    temp_sock.close(); # Just drop it
                else:
                    print("Accepting command socket.");
                    cmd_client_sock, cmd_client_addr = cmd_sock.accept();
                    cmd_connected = True;

            if curr_readable == data_sock:
                ###print("Data socket readable.");
                if data_connected: # Already have a conenction
                    temp_sock,temp_addr = data_sock.accept();
                    temp_sock.close(); # Just drop it
                else:
                    if cmd_connected == False:
                        continue;
                    print("Accepting new data connection.");
                    data_client_sock,temp_addr = data_sock.accept();
                    data_connected = True;

            if curr_readable == data_client_sock:
                #print("Data client readable.");
                # Just swallow the data
                temp_data = data_client_sock.recv(1024);
                if temp_data == b'': # Disconnection
                    data_client_sock.close();
                    data_client_sock = None;
                    data_connected = False;
                
            if curr_readable == cmd_client_sock:
                #print("Command client readable.");
                # Read the data
                cmd_read_data = cmd_client_sock.recv(1024); 
                cmd_get_time = time.monotonic();
                print("Received command: {}".format(cmd_read_data.decode()));
                bsharp_sock.send(cmd_read_data);
                # Do nothing 
            
            if curr_readable == bsharp_sock:

                if FLUSH:
                    dump = bsharp_sock.recv(4096); # Get partial read
                    dump_counter = dump_counter + 1
                    if internal_log != None:
                       internal_log.append("DC" + str(dump_counter))
                    continue

                #print("Can read bsharp socket");
                #curr_bsharp = bsharp_sock.recv(4096); # Check length
                curr_bsharp = bsharp_sock.recv(4096); # Get partial read
                from_bsharp_socket = from_bsharp_socket + curr_bsharp;

                # XXX FIXME This is to flush the buffer if we get out of sync.
                # The number should probably change.
                if ALLOW_FLUSH:
                    if (len(from_bsharp_socket)) > (440*2+32): # Two data transmissions plus a command
                        from_bsharp_socket = b'';
                        FLUSH = True;
                        FIFO_DIRTY = True;
                        ## YF[
                        select_timeout = 0.001;   # If packets are too rapid, clear quickly
                        print("Set Broadcast off")
                        bsharp_sock.send(b'bs 152 2\r\n');   # STOP Transmitting - Turn off Broadcast mode
                        continue

                ## YF]

        # Can only handle one packet per loop to avoid overwriting responses from the socket for the writable below
        if len(from_bsharp_socket) > 0:
            (read_status, read_bytes) = bsharp_all_recv(from_bsharp_socket);
            if read_status == RECV_PARTIAL:
                pass;           # Nothing to do here
            elif read_status == RECV_FLUSH:
                from_bsharp_socket = b''; # Need to flush the data
                FIFO_DIRTY = False;       # No need to print and empty FIFO
                select_timeout = 0.001;   # If packets are too rapid, clear quickly
            else:                         # Full received packet
                from_bsharp_socket = from_bsharp_socket[len(read_bytes):]; # Strip the valid data from the queue
                FIFO_DIRTY = True; # FIFO modified, so set dirty
                ###
                #print("Got a full packet");
                if (read_bytes[0] == ord(b'B')) and (read_bytes[1] == 1):
                    data_pending = DATA_CURR;
                    data_in_count = data_in_count +1;
                    ###
                    #print("Got a B1");
                elif (read_bytes.find(b'bb') == 0):
                    data_pending = DATA_CURR;
                    data_in_count = data_in_count + 1;
                else:           # A command
                    data_pending = DATA_CMD;
                    ###
                    print("Got a command");
                    try:
                        print("Received B# command response: {}".format(read_bytes.decode()));
                    except:
                        resp_string = "Received B# command response hex: ";
                        for curr_byte in read_bytes[0:64]:
                            resp_string += "{:02x} ".format(curr_byte);
                        print(resp_string);
                    # XXX Swallow broadcast enable/disable responses, as they will not come from EPICS

                    if (read_bytes.find(b'bs 152') == 0) or (read_bytes.find(b'bc 152') == 0):
                        read_bytes = b'';
                        data_pending = DATA_NONE;
                        
        for curr_writable in write_list: # TODO remove disconnected sockets from list
            if (curr_writable == data_client_sock) and (data_pending == DATA_CURR):
                curr_bsharp = read_bytes;
                packet_count = packet_count+1;
                # Check for proper header
                delimit_idx = curr_bsharp.find(b'>');
                #for idx in range(delimit_idx):
                #   print((curr_bsharp[idx]));

                #print("Find index is: {}\tText is {},{}.".format(curr_bsharp.find(b'rb'),chr(curr_bsharp[0]),curr_bsharp[1]));
                #print("Length is: {}".format(len(curr_bsharp)));
                if (curr_bsharp.find(b'bb') == 0) or (curr_bsharp.find(b'B\x01') == 0): # Found first header
                    if (curr_bsharp.find(b'bb') == 0):
                        delimit_idx = curr_bsharp.find(b'>');
                    else:
                        delimit_idx = -1; # Incremented below
                    bsharp_data = curr_bsharp[(delimit_idx+1):];
                    bytes_sent = data_client_sock.send(bsharp_data);
                    if (packet_count % 10)   ==   0:
                        pass
                        ###print("Sent {} bytes.".format(bytes_sent));
                        ###print("Data starts: {}, {}, {}, {}".format(bsharp_data[0], bsharp_data[1], bsharp_data[2], bsharp_data[3]));
                    data_pending = DATA_NONE;
                    data_out_count = data_out_count + 1;
            if (curr_writable == cmd_client_sock) and (data_pending == DATA_CMD):
                curr_bsharp = read_bytes;
                cmd_client_sock.send(curr_bsharp);
                cmd_finish_time = time.monotonic();
                print("Command took {} seconds.".format(cmd_finish_time-cmd_get_time));
                try:
                    print("Command response string: {}".format(curr_bsharp.decode()))
                except:
                    debug_string = "Command response hex: ";
                    for curr_byte in curr_bsharp:
                        debug_string = debug_string + "{:02x} ".format(curr_byte);
                    print(debug_string);
                    ###Below can error out if packets out of sync.
                ###print("Command client send string: {}".format(curr_bsharp.decode()));
                data_pending = DATA_NONE
        sys.stdout.flush();
except Exception as inst:

    traceback.print_exc();
    print(type(inst));
    print(inst);
    cmd_sock.close();
    data_sock.close();
