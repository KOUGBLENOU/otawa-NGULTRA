/*
 *	ultra_r5 module implementation
 *
 *	This file is part of OTAWA
 *	Copyright (c) 2017, IRIT UPS.
 *
 *	OTAWA is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	OTAWA is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with OTAWA; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include <otawa/events/StandardEventBuilder.h>
#include <otawa/etime/EdgeTimeBuilder.h>
#include <otawa/prop/DynIdentifier.h>
#include <elm/sys/Path.h>
#include <otawa/loader/arm.h>
#include <elm/io/FileOutput.h>
#include <elm/data/Vector.h>

namespace otawa { namespace ultra {
    using namespace elm::io;
	extern p::id<bool> WRITE_LOG;
	
    typedef enum {
        FE    = 0,
        DE    = 1,
        ISS   = 2,
        EXE   = 3,
        WR    = 4,
        CNT   = 5
    } pipeline_stage_t;

	
	class R52FExeGraph: public etime::EdgeTimeGraph {
	public:
		
		R52FExeGraph(WorkSpace* ws,
                 ParExeProc* proc, 
                 Vector<Resource* >* hw_resources, 
				 ParExeSequence* seq,
                 const PropList &props,
                 FileOutput* out, 
                 elm::Vector<Address>* unknown_inst_address) : etime::EdgeTimeGraph(ws, proc, hw_resources, seq, props), 
                                                                exec_int(0), exec_fpu_simd(0), _out(out), 
                                                                _unknown_inst_address(unknown_inst_address) {
			
			// Try to find arm loader with arm information
			DynIdentifier<arm::Info* > id("otawa::arm::Info::ID");
			info = id(_ws->process());
			if (!info)
				throw Exception("ARM loader with otawa::arm::INFO is required !");
			// Get memory configuration
			mem = hard::MEMORY_FEATURE.get(ws);
			ASSERTP(mem, "Memory feature not found");
		}


		void build(void) override {			
			// Look for FUs
			for (ParExePipeline::StageIterator pipeline_stage(_microprocessor->pipeline()); pipeline_stage(); pipeline_stage++) {
				if (pipeline_stage->name() == "PreFetch") {
					stage[FE] = *pipeline_stage;
				} else if (pipeline_stage->name() == "Decode") {
					stage[DE] = *pipeline_stage;
				} else if (pipeline_stage->name() == "Issue") {
                    stage[ISS] = *pipeline_stage;
                } else if (pipeline_stage->name() == "EXE") {
					// _microprocessor->setExecStage(*pipeline_stage);
					stage[EXE] = *pipeline_stage;
					for (int i = 0; i < pipeline_stage->numFus(); i++) {
						ParExePipeline* fu = pipeline_stage->fu(i);
						if (fu->firstStage()->name().startsWith("FPU_SIMD")) {
							exec_fpu_simd = fu;
						} else if (fu->firstStage()->name().startsWith("INTEGER")) {
							exec_int = fu;
						} else
							ASSERTP(false, fu->firstStage()->name() << ": Unknown functionnal-unit description found.");
					}
				} else if (pipeline_stage->name() == "Write") {
					stage[WR] = *pipeline_stage;
				} 

			}
			ASSERTP(stage[FE], "No 'Prefetch' stage found");
			ASSERTP(stage[DE], "No 'Decode' stage found");
            ASSERTP(stage[ISS], "No 'Issue' stage found");
			ASSERTP(stage[EXE], "No 'EXE' stage found");
			ASSERTP(stage[WR], "No 'Write back' stage found");
			ASSERTP(exec_int, "No 'INTEGER' fu found");
			ASSERTP(exec_fpu_simd, "No 'FPU/SIMD' fu found");
			

			// Build the execution graph 
			// createSequenceResources();
			// createNodes();
			// addEdgesForPipelineOrder();
			// addEdgesForFetch();
			// addEdgesForProgramOrder();
			// addEdgesForMemoryOrder();
			// addEdgesForDataDependencies();
			// dumpUnknowInst();
            ParExeGraph::build();
		}

		
	private:
		otawa::arm::Info* info;
		const hard::Memory* mem;
		ParExeStage* stage[CNT];
		ParExePipeline *exec_fpu_simd, *exec_int;
		FileOutput* _out = nullptr;
		elm::Vector<Address>* _unknown_inst_address = nullptr;
		/*
			Find the Mem stage of an instruction.
			    inst: Concerned instruction.
		*/
		ParExeNode* findMemStage(ParExeInst* inst) {
			for (ParExeInst::NodeIterator node(inst); node(); node++) {
					if (node->stage() == _microprocessor->memStage())
						return *node;
			}
			return nullptr;
		}

		/*
			Find the WB stage of an instruction.
			    inst: Concerned instruction.
		*/
		ParExeNode* findWrSstage(ParExeInst* inst) {
			for (ParExeInst::NodeIterator node(inst); node(); node++) {
					if (node->stage() == stage[WR])
						return *node;
			}
			return nullptr;
		}

	};


	class BBTimerNGUltraR52: public etime::EdgeTimeBuilder {
	public:
		static p::declare reg;
		BBTimerNGUltraR52(void): etime::EdgeTimeBuilder(reg) { }

	protected:
		virtual void configure(const PropList& props) {
			etime::EdgeTimeBuilder::configure(props);
			write_log = WRITE_LOG(props);
			_props = props;
		}
		void setup(WorkSpace* ws) override {
			etime::EdgeTimeBuilder::setup(ws);
			const hard::CacheConfiguration* cache_config = hard::CACHE_CONFIGURATION_FEATURE.get(ws);
			if (!cache_config)
				throw ProcessorException(*this, "no cache");

			if (!cache_config->hasDataCache())
				throw ProcessorException(*this, "no data cache");

			if (!cache_config->hasInstCache())
				throw ProcessorException(*this, "no instruction cache");

			if (cache_config->isUnified())
				throw ProcessorException(*this, "unified L1 cache not supported");
			if (write_log) {
				sys::Path log_file_path = sys::Path(ws->process()->program()->name() + ".log");
				bool write_header = (log_file_path.exists()) ? false : true;
				log_stream = new FileOutput(log_file_path, true);
				if (write_header)
					*log_stream << "########################################################" << endl
								<< "# Static analysis on " << ws->process()->program()->name() << endl
								<< "# Overestimated instructions" << endl
								<< "# Address (hex); Instruction" << endl
								<< "########################################################" << endl;
				else
					*log_stream << endl; // sep
				
				unknown_inst_address = new elm::Vector<Address>();
			}
		}

		etime::EdgeTimeGraph* make(ParExeSequence* seq) override {
			R52FExeGraph* graph = new R52FExeGraph(workspace(), _microprocessor, ressources(), seq, _props, log_stream, unknown_inst_address);
			graph->build();
			return graph;
		}


		virtual void clean(ParExeGraph* graph) {
			log_stream->flush();
			delete graph;
		}
	private:
		PropList _props;
		FileOutput* log_stream = nullptr;
		bool write_log = 0;
		elm::Vector<Address>* unknown_inst_address = nullptr;
	};

	

	p::declare BBTimerNGUltraR52::reg = p::init("otawa::ultra::BBTimerNGUltraR52", Version(1, 0, 0))
										.extend<etime::EdgeTimeBuilder>()
										.require(otawa::hard::CACHE_CONFIGURATION_FEATURE)
										.maker<BBTimerNGUltraR52>();
	

} // namespace ultra
} // namespace otawa
