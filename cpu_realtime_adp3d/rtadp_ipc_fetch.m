function result=rtadp_ipc_fetch(future)
    packet=load(future.result,'reply');
    if ~packet.reply.ok, error('RTADP:WorkerFailure','%s',packet.reply.message); end
    result=packet.reply.result;
end
