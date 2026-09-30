using Cavern.Format;
using Cavern.Format.Renderers;

const int blockFrames = 256;

if (args.Length != 1)
{
    Console.Error.WriteLine("Usage: AtmosDecoder <eac3-joc-file>");
    return 64;
}

try
{
    using var reader = AudioReader.Open(args[0]);
    reader.ReadHeader();
    using var renderer = reader.GetRenderer();
    if (renderer is not EnhancedAC3Renderer eac3Renderer || !renderer.HasObjects)
    {
        Console.Error.WriteLine("Input has no decoded E-AC-3 JOC objects.");
        return 2;
    }

    if (reader.Length <= 0)
    {
        Console.Error.WriteLine("Input duration is unknown.");
        return 3;
    }

    var objectCount = renderer.Objects.Count;
    var dynamicObjectCount = eac3Renderer.DynamicObjects;
    using var writer = new BinaryWriter(Console.OpenStandardOutput());
    writer.Write(new byte[] { (byte)'C', (byte)'A', (byte)'V', (byte)'O', (byte)'B', (byte)'J', (byte)'0', (byte)'1' });
    writer.Write(reader.SampleRate);
    writer.Write(objectCount);
    writer.Write(dynamicObjectCount);
    writer.Write(reader.Length);
    writer.Write(blockFrames);
    writer.Flush();

    for (long position = 0; position < reader.Length; position += blockFrames)
    {
        var frames = (int)Math.Min(blockFrames, reader.Length - position);
        var objectSamples = renderer.GetNextObjectSamples(frames);
        writer.Write(frames);
        for (var objectIndex = 0; objectIndex < objectCount; objectIndex++)
        {
            var source = renderer.Objects[objectIndex];
            var objectPosition = source.Position;
            var dynamicObject = objectIndex >= objectCount - dynamicObjectCount;
            var flags = (source.LFE ? 1 : 0) | (dynamicObject ? 2 : 0);
            writer.Write(flags);
            writer.Write(objectPosition.X);
            writer.Write(objectPosition.Y);
            writer.Write(objectPosition.Z);

            var samples = objectSamples[objectIndex];
            for (var frame = 0; frame < frames; frame++)
            {
                writer.Write(samples is not null && frame < samples.Length ? samples[frame] : 0.0f);
            }
        }
        writer.Flush();
    }

    return 0;
}
catch (Exception error)
{
    Console.Error.WriteLine(error.Message);
    return 1;
}
