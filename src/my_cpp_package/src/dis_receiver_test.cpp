#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// DIS library includes
#include <dis6/EntityStatePdu.h>
#include <dis6/utils/DataStream.h>

int main()
{
    std::cout << "Starting DIS receiver test on 127.0.0.1:3000..." << std::endl;
    
    // Create UDP socket
    int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return -1;
    }
    
    // Allow address reuse
    int opt = 1;
    setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    // Bind to loopback interface on port 3000
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    addr.sin_port = htons(3000);
    
    if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Failed to bind socket" << std::endl;
        close(socket_fd);
        return -1;
    }
    
    std::cout << "Listening for DIS packets on 127.0.0.1:3000..." << std::endl;
    
    char buffer[1500];
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);
    
    while (true) {
        ssize_t bytes_received = recvfrom(socket_fd, buffer, sizeof(buffer), 0, 
                                          (struct sockaddr*)&sender_addr, &sender_len);
        
        if (bytes_received > 0) {
            std::cout << "\nReceived " << bytes_received << " bytes from " 
                      << inet_ntoa(sender_addr.sin_addr) << ":" << ntohs(sender_addr.sin_port) << std::endl;
            
            try {
                // Try to parse as DIS EntityStatePdu using SetStream
                DIS::DataStream dataStream(DIS::BIG);
                dataStream.SetStream(buffer, bytes_received, DIS::BIG);
                
                DIS::EntityStatePdu entityPdu;
                entityPdu.unmarshal(dataStream);
                
                std::cout << "Successfully parsed EntityStatePdu:" << std::endl;
                std::cout << "  Entity ID: " << entityPdu.getEntityID().getEntity() << std::endl;
                std::cout << "  Force ID: " << static_cast<int>(entityPdu.getForceId()) << std::endl;
                std::cout << "  Position: (" 
                          << entityPdu.getEntityLocation().getX() << ", "
                          << entityPdu.getEntityLocation().getY() << ", "
                          << entityPdu.getEntityLocation().getZ() << ")" << std::endl;
                std::cout << "  Orientation: ("
                          << entityPdu.getEntityOrientation().getPsi() << ", "
                          << entityPdu.getEntityOrientation().getTheta() << ", "
                          << entityPdu.getEntityOrientation().getPhi() << ")" << std::endl;
                          
            } catch (const std::exception& e) {
                std::cout << "Error parsing DIS packet: " << e.what() << std::endl;
                std::cout << "Raw bytes: ";
                for (int i = 0; i < std::min((ssize_t)32, bytes_received); i++) {
                    printf("%02x ", (unsigned char)buffer[i]);
                }
                std::cout << std::endl;
            }
        }
    }
    
    close(socket_fd);
    return 0;
}