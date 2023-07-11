if((msg.payload>=0) && msg.payload<=5)
{
    msg.payload = 2;
    return msg;
}
else if(msg.payload>6)
{
     msg.payload = 0;
     return msg;
}
return msg;


SYS/relayio/0d2a4e71-e94e-2481-231e-fa160d97cbf5/set/sys