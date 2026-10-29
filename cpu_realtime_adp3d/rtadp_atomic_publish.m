function rtadp_atomic_publish(temporary,destination)
% Same-directory NTFS atomic rename without the general movefile command overhead.
% Readers only see a completed, closed MAT file. Each destination job name is unique.
    source=java.io.File(temporary).toPath();
    target=java.io.File(destination).toPath();
    options=javaArray('java.nio.file.CopyOption',1);
    options(1)=java.nio.file.StandardCopyOption.ATOMIC_MOVE;
    java.nio.file.Files.move(source,target,options);
end
